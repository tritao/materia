package processkit;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.event.EventValue;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.trajectory.ValidationLimits;
import processkit.WelderProcessDevice.WelderChannels;
import robotkit.manipulation.Manipulator;
import robotkit.skill.WeldPlan;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.WeldFault;
import robotkit.tool.WeldSensor;
import robotkit.tool.WeldSensor.WeldReading;
import robotkit.world.FiredProcessEvent;
import robotkit.world.Robot;

private enum WeldingPhase {
  Idle;
  /** The run has started and the device is being prepared. */
  Preparing;
  /** The program is running. */
  Welding;
  /** The arc was lost: the arm is stopping and the device clearing, before the restart. */
  Stopping;
  Done;
  Failed;
}

/** The welder's latest reading, which the device and the program's input wait both read. */
private class LatestReading implements WelderFeedback {
  public var value:WeldReading = {arc: false, currentA: 0.0, voltageV: 0.0, touch: false, fault: WeldFault.None, powerW: 0.0};

  public function new() {}

  public function reading():WeldReading return value;
}

/**
 * Welds one seam with an arm: a MotionKit program, run by `ManipulatorMotion`, whose process is driven by a
 * `ProcessRun` over a `WelderProcessDevice`. It works through the robot alone: the torch's channels and the arm's
 * motion, and the welder's `tool_weld` reading, which the caller hands in each tick (`WeldSeam` takes it from the
 * robot's sensor), so the same runner welds on a fixed arm or a mobile base, simulated or real.
 *
 * The program is: a joint move to the approach pose (`approach` out along the wire from the start), a straight move
 * to the start, then the process run's - the arc strikes there (voltage and wire speed set, arc on) and the program
 * waits on the welder's established arc, or gives the seam up after `IGNITION_TIMEOUT`; it dwells `startDwell`; the
 * torch follows the seam at the travel speed with the wire speed that keeps the deposit per length constant; it dwells
 * `craterDwell` at the end with the arc up; the wire stops and the torch lifts `LIFT` over the burnback time while the
 * arc, with nothing feeding it, burns back, and the arc command ends; the torch retracts to the approach pose. The
 * process run's engagement (`ProcessEngagement`) carries the arc and the crater.
 *
 * If the welder faults mid-seam - the arc is lost, or never strikes - the process run interrupts, the arm stops, and
 * once the fault has cleared the run restarts `BACKOFF` metres before where it stopped, so the new bead overlaps the
 * old; at most `maxRestarts` times, after which the weld fails.
 */
class WeldingPlanRunner implements robotkit.skill.WeldRunner {
  public static inline var FRAME:String = "arm-base";
  /** The name the program's input wait reads the established arc under. */
  public static inline var ARC_ESTABLISHED:String = "weld.arc_established";
  /** How long the program waits for the arc to establish, in seconds. */
  public static inline var IGNITION_TIMEOUT:Float = 2.0;
  /** How far before the stop a restart begins, in metres. */
  public static inline var BACKOFF:Float = 0.010;
  /** How far the torch lifts while the arc burns back, in metres. */
  public static inline var LIFT:Float = 0.005;
  /** Speed of the approach to the start and of the retract, in metres per second. */
  public static inline var APPROACH_SPEED:Float = 0.08;
  /** How long a fault may stay up after the arm has stopped before the weld is given up, in seconds. */
  public static inline var CLEAR_TIMEOUT:Float = 5.0;
  /** Cartesian accuracy the seam is followed to, in metres. */
  public static inline var PATH_TOLERANCE:Float = 0.0005;

  public final motion:ManipulatorMotion;
  public final channels:WelderChannels;
  public final maxRestarts:Int;
  /** The process run of the weld in progress. */
  public var current(default, null):Null<ProcessRun> = null;

  final outputs:ChannelWelderOutputs;
  final latest:LatestReading;
  var phase:WeldingPhase = Idle;
  var failureMessage:Null<String> = null;
  var plan:Null<WeldPlan> = null;
  var restartCount:Int = 0;
  var waiting:Float = 0.0;
  var followIndex:Int = -1;
  var reachedPath:Bool = false;
  var programStart:Float = 0.0;
  var seam:Null<PosePath> = null;
  var retreat:Null<Pose3> = null;

  /**
   * An arm's welding runner. `channels` are the torch's; `maxAcceleration` the joint acceleration programs plan with.
   */
  public static function create(robot:Robot, manipulator:Manipulator,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool}, channels:WelderChannels, maxAcceleration:Float,
      ?maxRestarts:Int = 3):WeldingPlanRunner {
    var count = manipulator.group.count();
    var limits = new ValidationLimits(count, Int64.ofInt(1), Int64.ofInt(0));
    for (joint in 0...count) {
      var bound = manipulator.group.limitsOf(joint);
      if (bound.lower < bound.upper) limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, bound.velocity > 0.0 ? bound.velocity : 2.0);
      limits.acceleration(joint, maxAcceleration);
      limits.jerk(joint, 20.0);
    }
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    var compiler = new ProgramCompiler(solver, limits, FRAME,
      [for (joint in 0...count) {
        var speed = manipulator.group.limitsOf(joint).velocity;
        speed > 0.0 ? speed : 2.0;
      }], [for (_ in 0...count) maxAcceleration], [for (_ in 0...count) 20.0],
      StartTolerances.uniform(count, 0.005, maxAcceleration * 0.01, 20.0 * 0.01),
      null, 0.002, 0.2, PATH_TOLERANCE, 0.02, new IkTolerance(2e-4, 1e-3, 300, 0.03));
    var indices = [for (target in manipulator.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    // The program waits on the established arc, which the welder's reading says.
    var latest = new LatestReading();
    var motion = new ManipulatorMotion(robot, compiler, function(channel) return channel == ARC_ESTABLISHED
      ? EventValue.Digital(latest.reading().arc) : null, eventSource, indices);
    return new WeldingPlanRunner(motion, channels, latest, maxRestarts);
  }

  /** Over an existing motion, whose input wait reads the established arc from `latest`. */
  function new(motion:ManipulatorMotion, channels:WelderChannels, latest:LatestReading, maxRestarts:Int) {
    if (motion == null || channels == null || latest == null || maxRestarts < 0)
      throw "WeldingPlanRunner needs motion, the torch's channels and a restart limit of zero or more";
    this.motion = motion;
    this.channels = channels;
    this.maxRestarts = maxRestarts;
    this.outputs = new ChannelWelderOutputs(channels);
    this.latest = latest;
  }

  public function restarts():Int return restartCount;
  public function running():Bool return phase == Preparing || phase == Welding || phase == Stopping;
  public function completed():Bool return phase == Done;
  public function failure():Null<String> return failureMessage;

  public function run(plan:WeldPlan):Void {
    if (running()) throw "A weld is already running";
    this.plan = plan;
    var parameters = plan.parameters;
    var start = pose(plan.start), stop = pose(plan.stop);
    var travel = parameters.travelSpeed;
    seam = new PosePath(FRAME, [new PoseLine(new PoseWaypoint(start, PATH_TOLERANCE, 0.01),
      new PoseWaypoint(stop, PATH_TOLERANCE, 0.01), OrientationPolicy.Interpolated, 0.1, travel)]);
    retreat = along(stop, plan.stop, -parameters.approach);
    // Arc on with the wire at the weld speed, held until the arc is established, then the start dwell.
    var entry:Array<MotionOp> = [
      MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(parameters.wireSpeed)),
      MotionOp.SetOutput(channels.arc, EventValue.Digital(true)),
      MotionOp.WaitInput(ARC_ESTABLISHED, InputPredicate.Equals(EventValue.Digital(true)), IGNITION_TIMEOUT)
    ];
    if (parameters.startDwell > 0) entry.push(MotionOp.Dwell(parameters.startDwell));
    // Crater fill with the arc up, then the wire stops; the torch lifts while the arc burns back, and the command ends.
    var exit:Array<MotionOp> = [];
    if (parameters.craterDwell > 0) exit.push(MotionOp.Dwell(parameters.craterDwell));
    exit.push(MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(0.0)));
    if (parameters.burnback > 0) {
      exit.push(MotionOp.MoveL(along(stop, plan.stop, -LIFT), FRAME, Math.max(LIFT / parameters.burnback, 0.01), Blend.ExactStop));
    }
    exit.push(MotionOp.SetOutput(channels.arc, EventValue.Digital(false)));
    var recipe = new ProcessRecipe(travel * 0.5, travel * 2.0, travel, 0.0, OrientationPolicy.Interpolated, 0.001,
      parameters.wireSpeed / travel, 0.0, BACKOFF, FeedChangePolicy.Reject, new ProcessEngagement(entry, exit), APPROACH_SPEED);
    var device = new WelderProcessDevice(outputs, latest, channels, {voltage: parameters.voltage});
    var process = new ProcessRun(recipe, seam, device, channels.wireSpeed, motion.session);
    current = process;
    process.start();
    restartCount = 0;
    failureMessage = null;
    waiting = 0.0;
    phase = Preparing;
  }

  public function update(dtSeconds:Float, reading:WeldReading):Void {
    latest.value = reading;
    var process = current;
    if (process == null) return;
    try {
      switch phase {
        case Idle | Done | Failed:
        case Preparing:
          process.update(0.0);
          if (process.state == ProcessRunState.Ready) launch(true);
        case Welding:
          motion.update(dtSeconds);
          var problem = motion.failure;
          if (problem != null && !motion.running) {
            // The program failed on its own: an arc that never established is the welder's to retry, anything else is not.
            if (problem.indexOf(ARC_ESTABLISHED) >= 0) {
              process.interruptNow(travelled(), "the arc did not establish");
              beginStop();
            } else
              fail(problem);
          } else if (problem == null) {
            process.update(travelled());
            if (process.state == ProcessRunState.ControlledInterruption) beginStop();
            else if (motion.completed) {
              process.finish();
              phase = Done;
            }
          }
        case Stopping:
          if (motion.running) motion.update(dtSeconds);
          waiting += dtSeconds;
          process.update(process.interruptedAt);
          if (process.state == ProcessRunState.Recovery && !motion.running) launch(false);
          else if (waiting > CLEAR_TIMEOUT)
            fail('the welder did not clear: ${WeldSensor.faultMessage(reading.fault)}');
      }
    } catch (error:Dynamic) {
      fail(Std.string(error));
    }
  }

  public function abort():Void {
    if (!running()) return;
    motion.abort();
    phase = Idle;
  }

  /** The arm stops and the weld waits for the welder to clear, to restart. */
  function beginStop():Void {
    restartCount++;
    if (restartCount > maxRestarts) {
      fail('the arc could not be held in $maxRestarts restarts');
      return;
    }
    if (motion.running) motion.abort();
    waiting = 0.0;
    phase = Stopping;
  }

  /** Starts the process run's program, the first time from a joint move to the approach pose. */
  function launch(first:Bool):Void {
    var process = cast(current, ProcessRun);
    var body = process.takeProgram();
    var ops:Array<MotionOp> = outputs.drain();
    if (first) {
      var current = cast(plan, WeldPlan);
      ops.push(MotionOp.MoveJ(MoveTarget.PoseTarget(along(pose(current.start), current.start, -current.parameters.approach), FRAME,
        null), new MotionOptions(), Blend.ExactStop));
    }
    followIndex = ops.length + process.followOp;
    ops = ops.concat(body.ops);
    ops.push(MotionOp.MoveL(cast(retreat, Pose3), FRAME, APPROACH_SPEED, Blend.ExactStop));
    programStart = process.lastProgramStart;
    reachedPath = false;
    waiting = 0.0;
    motion.run(new MotionProgram(ops));
    phase = Welding;
  }

  /** The seam distance the torch has reached: where the program is on the path, before and after it the start and the end. */
  function travelled():Float {
    var length = cast(seam, PosePath).length();
    var progress = motion.progress();
    if (progress.op == followIndex) {
      reachedPath = true;
      return Math.min(length, Math.max(0.0, programStart + progress.pathDistance));
    }
    return reachedPath ? length : programStart;
  }

  function fail(message:String):Void {
    try motion.abort() catch (_:Dynamic) {}
    failureMessage = message;
    phase = Failed;
  }

  static function pose(frame:Transform3):Pose3 {
    var t = frame.translation, r = frame.rotation;
    return new Pose3(t.x, t.y, t.z, r.x, r.y, r.z, r.w);
  }

  /** `from` moved `distance` metres along its +Z, the wire. */
  static function along(from:Pose3, frame:Transform3, distance:Float):Pose3 {
    var wire = frame.rotation.rotate(new Vec3(0.0, 0.0, 1.0));
    return new Pose3(from.x + wire.x * distance, from.y + wire.y * distance, from.z + wire.z * distance, from.qx, from.qy, from.qz,
      from.qw);
  }
}

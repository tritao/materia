package processkit;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.event.EventValue;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import robotkit.spatial.Quat;
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
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.Manipulator;
import processkit.skill.WeldPlan;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import processkit.tool.WeldFault;
import processkit.tool.WeldSensor;
import processkit.tool.WeldSensor.WeldReading;
import robotkit.execution.FiredProcessEvent;
import robotkit.core.Robot;

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
class WeldingPlanRunner implements processkit.skill.WeldRunner {
  public static inline var FRAME:String = "arm-base";
  /** The name the program's input wait reads the established arc under. */
  public static inline var ARC_ESTABLISHED:String = "weld.arc_established";
  /** How long the program waits for the arc to establish, in seconds. */
  public static inline var IGNITION_TIMEOUT:Float = 2.0;
  /** How far before the stop a restart begins, in metres. */
  public static inline var BACKOFF:Float = 0.010;
  /** How far the torch lifts while the arc burns back, in metres. */
  public static inline var LIFT:Float = WeldPathPlanner.LIFT;
  /** Speed of the approach to the start and of the retract, in metres per second. */
  public static inline var APPROACH_SPEED:Float = 0.08;
  /** How long the welder may take to become ready before the weld is given up, in seconds. */
  public static inline var PREPARE_TIMEOUT:Float = 2.0;
  /** How long a fault may stay up after the arm has stopped before the weld is given up, in seconds. */
  public static inline var CLEAR_TIMEOUT:Float = 5.0;
  /** The most the torch turns within one straight piece of a corner's turn, in radians. */
  public static inline var TURN_STEP:Float = 0.2;
  /** Cartesian accuracy the seam is followed to, in metres. */
  public static inline var PATH_TOLERANCE:Float = 0.0005;
  public final motion:ManipulatorMotion;
  public final channels:WelderChannels;
  public final maxRestarts:Int;
  /** The process run of the weld in progress. */
  public var current(default, null):Null<ProcessRun> = null;

  final planner:WeldPathPlanner;
  final wrist:WristLimits;
  final outputs:ChannelWelderOutputs;
  final latest:LatestReading;
  var phase:WeldingPhase = Idle;
  var failureMessage:Null<String> = null;
  var plan:Null<WeldPlan> = null;
  var restartCount:Int = 0;
  var waiting:Float = 0.0;
  var followIndex:Int = -1;
  /** Index, in the program, of the wait for the arc to establish. */
  var igniteIndex:Int = -1;
  var reachedPath:Bool = false;
  var programStart:Float = 0.0;
  var seam:Null<PosePath> = null;
  var retreat:Null<Pose3> = null;
  var planned:Null<PlannedWeld> = null;
  /** How long the last weld took to plan, in seconds, and what the plan is (for reports). */
  public var planningSeconds(default, null):Float = 0.0;
  public function lastPlan():Null<PlannedWeld> return planned;

  /** The weld's complete motion, validated without arc outputs or robot submission. Exact stops match the process program. */
  static function compileWeld(compiler:ProgramCompiler, wrist:WristLimits, planned:PlannedWeld,
      start:Array<Float>):motionkit.robot.CompiledProgram {
    var plan = planned.plan;
    var ops:Array<MotionOp> = [MotionOp.MoveJ(MoveTarget.JointTarget(planned.entry.joints), new MotionOptions(), Blend.ExactStop)];
    for (move in planned.entry.moves) ops.push(MotionOp.MoveL(move, FRAME, APPROACH_SPEED, Blend.ExactStop));
    ops.push(MotionOp.MoveL(pose(plan.start()), FRAME, APPROACH_SPEED, Blend.ExactStop));
    if (plan.parameters.startDwell > 0) ops.push(MotionOp.Dwell(plan.parameters.startDwell));
    ops.push(MotionOp.FollowPath(new PosePath(FRAME, pathOf(plan, plan.parameters.travelSpeed, wrist, planned.styles)), FRAME,
      plan.parameters.travelSpeed, []));
    if (plan.parameters.craterDwell > 0) ops.push(MotionOp.Dwell(plan.parameters.craterDwell));
    ops.push(MotionOp.MoveL(along(pose(plan.stop()), plan.stop(), -LIFT), FRAME,
      Math.max(LIFT / plan.parameters.burnback, 0.01), Blend.ExactStop));
    ops.push(MotionOp.MoveL(planned.retreat, FRAME, APPROACH_SPEED, Blend.ExactStop));
    return compiler.compile(new MotionProgram(ops), start, Int64.ofInt(1));
  }

  /**
   * An arm's welding runner. `channels` are the torch's; `maxAcceleration` the joint acceleration programs plan with.
   * `clearance`, when given, is what the welds are planned to be clear of the work with (`WeldPathPlanner`).
   */
  public static function create(robot:Robot, manipulator:Manipulator,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool}, channels:WelderChannels, maxAcceleration:Float,
      ?maxRestarts:Int = 3, ?clearance:ArmClearance):WeldingPlanRunner {
    var count = manipulator.group.count();
    var limits = new ValidationLimits(count, Int64.ofInt(1), Int64.ofInt(0));
    for (joint in 0...count) {
      var bound = manipulator.group.limitsOf(joint);
      if (bound.lower < bound.upper) limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, bound.velocity != null ? bound.velocity : 2.0);
      limits.acceleration(joint, maxAcceleration);
      limits.jerk(joint, 20.0);
    }
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    var compiler = new ProgramCompiler(solver, limits, FRAME,
      [for (joint in 0...count) {
        var speed = manipulator.group.limitsOf(joint).velocity;
        speed != null ? speed : 2.0;
      }], [for (_ in 0...count) maxAcceleration], [for (_ in 0...count) 20.0],
      StartTolerances.uniform(count, 0.005, maxAcceleration * 0.01, 20.0 * 0.01),
      null, 0.002, 0.2, PATH_TOLERANCE, 0.02, new IkTolerance(5e-5, 1e-3, 300, 0.03));
    var indices = [for (target in manipulator.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    // The program waits on the established arc, which the welder's reading says.
    var latest = new LatestReading();
    var motion = new ManipulatorMotion(robot, compiler, function(channel) return channel == ARC_ESTABLISHED
      ? EventValue.Digital(latest.reading().arc) : null, eventSource, indices);
    // The torch turns at corners as the wrist allows: the slowest of its last three joints, and the programs' acceleration.
    var wristSpeed = Math.POSITIVE_INFINITY;
    for (joint in Std.int(Math.max(0, count - 3))...count) {
      var speed = manipulator.group.limitsOf(joint).velocity;
      wristSpeed = Math.min(wristSpeed, speed != null && speed > 0.0 ? speed : 2.0);
    }
    var wrist:WristLimits = {angularSpeed: wristSpeed, angularAcceleration: maxAcceleration};
    var planner = new WeldPathPlanner(compiler.solver, compiler.ikTolerance, compiler.maxVelocity, wrist, clearance, compiler.perJointMaxJump,
      function(planned, start) return compileWeld(compiler, wrist, planned, start));
    return new WeldingPlanRunner(motion, channels, latest, maxRestarts, planner, wrist);
  }

  /** Over an existing motion, whose input wait reads the established arc from `latest`. */
  function new(motion:ManipulatorMotion, channels:WelderChannels, latest:LatestReading, maxRestarts:Int, planner:WeldPathPlanner,
      wrist:WristLimits) {
    if (motion == null || channels == null || latest == null || maxRestarts < 0 || planner == null || wrist == null)
      throw "WeldingPlanRunner needs motion, the torch's channels, a restart limit of zero or more, a planner and the wrist's limits";
    this.planner = planner;
    this.wrist = wrist;
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

  public function run(requested:WeldPlan):Void {
    if (running()) throw "A weld is already running";
    // Plan first: the rolls of the torch, how it comes in and leaves, reach and clearance all along. A weld that cannot be
    // done fails here, with the reasons.
    var began = Sys.time();
    var planned = planner.plan(requested, startPositions());
    planningSeconds = Sys.time() - began;
    this.planned = planned;
    var plan = planned.plan;
    this.plan = plan;
    var parameters = plan.parameters;
    var stop = pose(plan.stop());
    var travel = parameters.travelSpeed;
    seam = new PosePath(FRAME, pathOf(plan, travel, wrist, planned.styles));
    retreat = planned.retreat;
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
      exit.push(MotionOp.MoveL(along(stop, plan.stop(), -LIFT), FRAME, Math.max(LIFT / parameters.burnback, 0.01), Blend.ExactStop));
    }
    exit.push(MotionOp.SetOutput(channels.arc, EventValue.Digital(false)));
    var recipe = new ProcessRecipe(travel * 0.5, travel * 2.0, travel, 0.0, OrientationPolicy.Interpolated, 0.001,
      parameters.wireSpeed / travel, 0.0, BACKOFF, FeedChangePolicy.Reject, new ProcessEngagement(entry, exit), APPROACH_SPEED,
      PREPARE_TIMEOUT);
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
          // A welder with a fault it holds (a wire stuck to the work) never becomes ready: say so rather than wait for ever.
          process.update(0.0, dtSeconds);
          if (process.state == ProcessRunState.Ready) launch(true);
          else if (process.state == ProcessRunState.Failed) fail(cast(process.failure, String));
        case Welding:
          motion.update(dtSeconds);
          var problem = motion.failure;
          if (problem != null && !motion.running) {
            // The program failed on its own: an arc that never established (it failed waiting for it) is the welder's
            // to retry, anything else is not.
            if (motion.progress().op == igniteIndex || problem.indexOf(ARC_ESTABLISHED) >= 0) {
              process.interruptNow(travelled(), "the arc did not establish");
              beginStop();
            } else
              fail(problem);
          } else if (problem == null) {
            // Once past the path the seam is welded: a fault in the crater, the burnback or the lift is not a reason to
            // travel it again, only to finish ending the arc, which the rest of the program does and which clears it.
            if (!pastPath()) {
              process.update(travelled());
              if (process.state == ProcessRunState.ControlledInterruption) {
                beginStop();
                return;
              }
            }
            if (motion.completed) {
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
    var retreatMove = MotionOp.MoveL(cast(retreat, Pose3), FRAME, APPROACH_SPEED, Blend.ExactStop);
    var body = process.takeProgram(retreatMove);
    var ops:Array<MotionOp> = outputs.drain();
    if (first) {
      var current = cast(plan, WeldPlan);
      var entry = cast(planned, PlannedWeld).entry;
      ops.push(MotionOp.MoveJ(MoveTarget.JointTarget(entry.joints), new MotionOptions(), Blend.ExactStop));
      for (move in entry.moves) ops.push(MotionOp.MoveL(move, FRAME, APPROACH_SPEED, Blend.ExactStop));
    }
    followIndex = ops.length + process.followOp;
    // The entry is the two outputs, the wait for the arc, and perhaps a dwell, just before the path.
    igniteIndex = followIndex - (cast(plan, WeldPlan).parameters.startDwell > 0 ? 2 : 1);
    ops = ops.concat(body.ops);
    programStart = process.lastProgramStart;
    reachedPath = false;
    waiting = 0.0;
    var program = new MotionProgram(ops);
    // Joint limits can slow a corner below the requested feed. Schedule wire quantity against that
    // validated clock before submitting the program, so the device applies it with the motion.
    var timed = motion.compiler.compile(program, startPositions(), haxe.Int64.ofInt(1));
    var scheduled:MotionProgram;
    try {
      scheduled = ProcessRateSchedule.apply(program, timed, followIndex, channels.wireSpeed,
        process.recipe.quantityPerDistance, cast(plan, WeldPlan).parameters.wireSpeed);
    } catch (error:Dynamic) {
      timed.dispose();
      throw error;
    }
    timed.dispose();
    motion.run(scheduled);
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

  /** The program is past the path, in the crater fill, burnback, lift or retreat. */
  function pastPath():Bool {
    var op = motion.progress().op;
    if (op == followIndex) reachedPath = true;
    return reachedPath && (op > followIndex || op < 0);
  }

  function fail(message:String):Void {
    try motion.abort() catch (_:Dynamic) {}
    failureMessage = message;
    phase = Failed;
  }

  /** Where the arm's joints are now, or will be when the program running ends: where the next program starts. */
  function startPositions():Array<Float> {
    var commanded = motion.commandedPositions();
    if (commanded != null) return commanded;
    var positions = motion.robot.snapshot().positions;
    return [for (index in motion.jointIndices) positions.get(index)];
  }

  /**
   * The path the wire tip follows: a straight line per segment, the torch holding its orientation along it. Where
   * the next segment's orientation differs (a corner), the torch turns as it passes: along the last stretch of one
   * segment and the first of the next it turns half the way each, so that it meets the corner at the orientation midway
   * between the two and has the next segment's own orientation once past it. The turn is part of the travel, at the
   * travel speed, so the wire feed that keeps the deposit per length constant stays right, and no metal is piled
   * where the torch would otherwise stand still to turn. How long the stretch is comes from the angle and the wrist's
   * limits (`WeldCorner`), and a corner takes at most `WeldCorner.SHARE` of a segment.
   */
  static function pathOf(plan:WeldPlan, travel:Float, wrist:WristLimits, styles:Array<Int>):Array<PosePrimitive> {
    var segments = plan.segments;
    var primitives:Array<PosePrimitive> = [];
    function waypoint(point:Vec3, rotation:Quat):PoseWaypoint
      return new PoseWaypoint(new Pose3(point.x, point.y, point.z, rotation.x, rotation.y, rotation.z, rotation.w), PATH_TOLERANCE, 0.01);
    function line(from:Vec3, fromRotation:Quat, to:Vec3, toRotation:Quat):Void
      primitives.push(new PoseLine(waypoint(from, fromRotation), waypoint(to, toRotation), OrientationPolicy.Interpolated, 0.1, travel));
    /** A stretch of a turn from `r1` to `r2` of `angle` radians, taking it from fraction `s0` to `s1`, in steps of at most TURN_STEP radians. */
    function turn(from:Vec3, to:Vec3, r1:Quat, r2:Quat, s0:Float, s1:Float, angle:Float, travelA:Vec3, travelB:Vec3, style:Int):Void {
      var steps = Std.int(Math.max(1.0, Math.ceil(angle * (s1 - s0) / TURN_STEP)));
      for (step in 0...steps) {
        var f0 = step / steps, f1 = (step + 1) / steps;
        line(from.add(to.sub(from).scale(f0)), WeldCorner.orientationAt(r1, r2, s0 + (s1 - s0) * f0, travelA, travelB, style),
          from.add(to.sub(from).scale(f1)), WeldCorner.orientationAt(r1, r2, s0 + (s1 - s0) * f1, travelA, travelB, style));
      }
    }
    for (index in 0...segments.length) {
      var segment = segments[index];
      var a = segment.start.translation, b = segment.stop.translation;
      var length = segment.length();
      var direction = b.sub(a).scale(1.0 / length);
      var startRotation = segment.start.rotation, stopRotation = segment.stop.rotation;
      var beforeAngle = index > 0 ? segments[index - 1].stop.rotation.angularDistance(startRotation) : 0.0;
      var afterAngle = index + 1 < segments.length ? stopRotation.angularDistance(segments[index + 1].start.rotation) : 0.0;
      var before = beforeAngle > WeldCorner.STRAIGHT;
      var after = afterAngle > WeldCorner.STRAIGHT;
      var rampIn = before ? WeldCorner.given(WeldCorner.turnLength(beforeAngle, travel, wrist), length) : 0.0;
      var rampOut = after ? WeldCorner.given(WeldCorner.turnLength(afterAngle, travel, wrist), length) : 0.0;
      var from = a;
      if (before) {
        // The second half of the turn from the segment before.
        var end = a.add(direction.scale(rampIn));
        turn(a, end, segments[index - 1].stop.rotation, startRotation, 0.5, 1.0, beforeAngle, directionOf(segments[index - 1]), direction, styles[index]);
        from = end;
      }
      var until = after ? b.sub(direction.scale(rampOut)) : b;
      if (until.sub(from).norm() > 1e-6) line(from, startRotation, until, stopRotation);
      // The first half of the turn to the segment after.
      if (after) turn(until, b, stopRotation, segments[index + 1].start.rotation, 0.0, 0.5, afterAngle, direction, directionOf(segments[index + 1]), styles[index + 1]);
    }
    return primitives;
  }

  /** The unit direction a segment runs in. */
  static function directionOf(segment:WeldSegment):Vec3
    return segment.stop.translation.sub(segment.start.translation).normalized();

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

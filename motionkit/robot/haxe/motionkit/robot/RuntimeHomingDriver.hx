package motionkit.robot;

import haxe.Int64;
import motionkit.AxisTarget;
import motionkit.MotionOptions;
import motionkit.axis.MotionAxis;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.trajectory.Trajectory;
import robotkit.core.Robot;
import robotkit.core.RobotCommand;
import robotkit.core.StopMode;
import robotkit.execution.ExecutionPlanPurpose;
import robotkit.runtime.RobotRuntime;
import motionkit.robot.HomingDriver.HomingObservation;

/** Execute seek/backoff/approach as bounded homing plans and native controlled stops. */
class RuntimeHomingDriver implements HomingDriver {
  static var nextTag:Int64 = Int64.ofInt(200000000);
  final robot:Robot;
  final runtime:RobotRuntime;
  final observer:RuntimeHomingObserver;
  final planner:AxisPlanner;
  final axes:Map<Int, HomingAxis> = new Map();
  final afterLatch:Void -> Void;

  /** afterLatch resets the caller-owned encoder and stepper-slip monitors. */
  public function new(robot:Robot, runtime:RobotRuntime, homes:Array<HomingAxis>,
      motionAxes:Array<MotionAxis>, afterLatch:Void -> Void) {
    if (robot == null || runtime == null || homes == null || homes.length == 0 ||
        motionAxes == null || afterLatch == null) throw "Homing driver requires runtime, axes and latch monitor reset";
    this.robot = robot; this.runtime = runtime; this.afterLatch = afterLatch;
    var expanded:Array<MotionAxis> = [];
    var names = robot.description().joints;
    if (homes[0] == null) throw "Homing driver has a null home axis";
    var timestep = homes[0].timestep;
    for (home in homes) {
      if (home == null || axes.exists(home.joint) || home.timestep != timestep)
        throw "Homing driver requires distinct axes on one owner timestep";
      axes.set(home.joint, home);
    }
    for (axis in motionAxes) {
      if (axis == null) throw "Homing driver has a null motion axis";
      var home = axes.get(axis.jointIndices[0]);
      if (home != null && (axis.id != home.id || axis.jointScale(0) != 1.0 || axis.jointOffset(0) != 0.0))
        throw "Homing requires the independent joint in SI coordinates";
      expanded.push(new MotionAxis(new MotionAxisBlueprint(axis.id, axis.jointIds,
        home == null ? axis.lowerLimit : home.lowerTravel,
        home == null ? axis.upperLimit : home.upperTravel,
        axis.maxVelocity, axis.maxAcceleration, axis.homePosition,
        [for (i in 0...axis.jointIndices.length) axis.jointScale(i)],
        [for (i in 0...axis.jointIndices.length) axis.jointOffset(i)]), names));
    }
    for (home in homes) {
      var matched = false;
      for (axis in expanded) if (axis.jointIndices[0] == home.joint) matched = true;
      if (!matched) throw "Homing driver has no physical motion mapping";
    }
    planner = new AxisPlanner(expanded, timestep);
    // Runtime positions are already logical; captured edges stay endpoint coordinates.
    observer = new RuntimeHomingObserver(robot, homes, (joint, position) -> position,
      (joint, edge) -> edge);
  }

  public function observe(joint:Int):HomingObservation return observer.observe(joint);

  public function velocity(joint:Int, velocity:Float, acceleration:Float):Void {
    var axis = requireAxis(joint);
    if (!Math.isFinite(velocity) || velocity == 0.0) throw "Homing seek needs nonzero finite velocity";
    move(axis, velocity < 0.0 ? axis.lowerTravel : axis.upperTravel, Math.abs(velocity), acceleration);
  }

  public function stop(joint:Int, acceleration:Float):Void {
    requireAxis(joint);
    robot.stop(StopMode.Normal);
  }

  public function latch(switchId:String, counterPosition:Float):Void {
    runtime.latchHome(switchId, counterPosition);
    afterLatch();
  }

  public function returnHome(joint:Int, position:Float, velocity:Float, acceleration:Float):Void
    move(requireAxis(joint), position, velocity, acceleration);

  function requireAxis(joint:Int):HomingAxis {
    var axis = axes.get(joint);
    if (axis == null) throw "Unknown homing drive joint";
    return axis;
  }

  function move(axis:HomingAxis, position:Float, velocity:Float, acceleration:Float):Void {
    var snapshot = robot.snapshot();
    // Native plans anchor on held commanded coordinates, including measured following error.
    var trajectory = planner.plan(snapshot.setpointPositions.toArray(),
      {targets: [new AxisTarget(axis.id, position)], options: new MotionOptions(velocity, acceleration)}).trajectory;
    try {
      var stream = new TrajectoryStream(robot);
      var segments = trajectory.segments();
      stream.begin(segments, trajectory.durationSeconds());
      var native = runtime.snapshot();
      var tag = nextTag; nextTag = Int64.add(nextTag, Int64.ofInt(1));
      var tolerances = [for (_ in snapshot.positions.toArray()) 1e-6];
      var plan = stream.motionSubmission(trajectory, 0, segments.length, tag,
        Int64.ofInt(0), native.modelRevision, native.calibrationRevision,
        false, tolerances, ExecutionPlanPurpose.Homing);
      robot.submit(RobotCommand.ExecutionPlan(plan));
    } catch (error:Dynamic) {
      trajectory.dispose();
      throw error;
    }
    trajectory.dispose();
  }
}

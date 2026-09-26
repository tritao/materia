package motionkit.axis;

import motionkit.Feed;
import motionkit.path.PathPoint;
import motionkit.planner.TrajectoryPlanner;
import motionkit.planner.TrapezoidalPlanner;
import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;
import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.StopMode;

/**
 * Semantic machine-axis view over an ordinary RobotKit robot.
 *
 * The view owns planning state, while the Robot still owns snapshots,
 * transport, safety, and lifecycle. The bootstrap submits one position batch
 * per deterministic update; a buffered RobotCommand is a later execution
 * increment behind this same API.
 */
class MotionSystem {
  public final robot:Robot;
  public final axes:Array<MotionAxis>;
  public final fixedTimestepSeconds:Float;
  public final planner:TrajectoryPlanner;
  var activeTrajectory:Null<JointTrajectory> = null;
  var elapsedSeconds:Float = 0.0;

  public static function fromBlueprint(robot:Robot, blueprint:MotionSystemBlueprint):MotionSystem
    return new MotionSystem(robot, blueprint);

  public function new(robot:Robot, blueprint:MotionSystemBlueprint,
      ?planner:TrajectoryPlanner) {
    if (robot == null || blueprint == null) throw "Motion system needs a robot and blueprint";
    var description = robot.description();
    if (description == null || description.joints.length != blueprint.model.joints.length)
      throw "Robot description does not match the motion-system model";
    for (i in 0...blueprint.model.joints.length) {
      if (description.joints[i] != blueprint.model.joints[i].name)
        throw 'Robot joint $i does not match motion-system joint "${blueprint.model.joints[i].name}"';
    }
    this.robot = robot;
    this.fixedTimestepSeconds = blueprint.fixedTimestepSeconds;
    this.planner = planner == null ? new TrapezoidalPlanner(fixedTimestepSeconds) : planner;
    this.axes = [];
    var axisIds = new Map<String, Bool>();
    for (axisBlueprint in blueprint.axes) {
      var axis = new MotionAxis(axisBlueprint, description.joints);
      if (axisIds.exists(axis.id)) throw 'Duplicate motion axis "${axis.id}"';
      axisIds.set(axis.id, true);
      this.axes.push(axis);
    }
  }

  public function axis(id:String):Null<MotionAxis> {
    for (value in axes) if (value.id == id) return value;
    return null;
  }

  public function isMoving():Bool return activeTrajectory != null;

  public function trajectory():Null<JointTrajectory> return activeTrajectory;

  /** Plans a coordinated move while leaving unspecified axes at their current positions. */
  public function moveAxes(targets:Array<AxisTarget>, ?options:MotionOptions):JointTrajectory {
    if (targets == null || targets.length == 0) throw "Axis move needs at least one target";
    var snapshot = robot.snapshot();
    var start = snapshot.positions.toArray();
    var goal = start.copy();
    var seen = new Map<String, Bool>();
    for (target in targets) {
      if (target == null) throw "Axis move cannot contain a null target";
      var axisValue = axis(target.axis);
      if (axisValue == null) throw 'Unknown motion axis "${target.axis}"';
      if (seen.exists(target.axis)) throw 'Axis move targets "${target.axis}" more than once';
      if (target.position < axisValue.lowerLimit || target.position > axisValue.upperLimit)
        throw 'Axis "${target.axis}" target ${target.position} is outside its limits';
      axisValue.writeLogicalPosition(goal, target.position);
      seen.set(target.axis, true);
    }
    var chosenOptions = options == null ? new MotionOptions() : options;
    var limits = resolveLimits(targets, chosenOptions);
    activeTrajectory = planner.plan(start, goal, limits);
    elapsedSeconds = 0.0;
    return activeTrajectory;
  }

  /** Plans a straight Cartesian move for a direct XYZ gantry. */
  public function moveLinear(target:PathPoint, feed:Feed):JointTrajectory {
    if (target == null || feed == null) throw "Linear move needs a target and feed";
    var xAxis = requireAxis("x");
    var yAxis = requireAxis("y");
    var zAxis = requireAxis("z");
    var snapshot = robot.snapshot();
    var start = snapshot.positions.toArray();
    var goal = start.copy();
    var starts = [xAxis.logicalPosition(start), yAxis.logicalPosition(start),
      zAxis.logicalPosition(start)];
    var goals = [target.x, target.y, target.z];
    var axes = [xAxis, yAxis, zAxis];
    for (i in 0...3) {
      if (goals[i] < axes[i].lowerLimit || goals[i] > axes[i].upperLimit)
        throw 'Axis "${axes[i].id}" target ${goals[i]} is outside its limits';
      axes[i].writeLogicalPosition(goal, goals[i]);
    }

    var dx = goals[0] - starts[0];
    var dy = goals[1] - starts[1];
    var dz = goals[2] - starts[2];
    var distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (distance <= 0.0) {
      activeTrajectory = new JointTrajectory([new JointTrajectorySample(0.0, goal)]);
      elapsedSeconds = 0.0;
      return activeTrajectory;
    }

    var scalarFeed = feed.value;
    var scalarAcceleration = 1.0;
    for (i in 0...3) {
      var displacement = Math.abs(goals[i] - starts[i]);
      if (displacement <= 0.0) continue;
      if (axes[i].maxVelocity > 0.0)
        scalarFeed = Math.min(scalarFeed, axes[i].maxVelocity * distance / displacement);
      if (axes[i].maxAcceleration > 0.0)
        scalarAcceleration = Math.min(scalarAcceleration,
          axes[i].maxAcceleration * distance / displacement);
    }

    var scalarTrajectory = planner.plan([0.0], [distance],
      new MotionLimits(scalarFeed, scalarAcceleration));
    var mapped:Array<JointTrajectorySample> = [];
    for (sample in scalarTrajectory.samples) {
      var alpha = sample.positions[0] / distance;
      var positions:Array<Float> = [];
      var velocities:Array<Float> = [];
      var accelerations:Array<Float> = [];
      for (j in 0...start.length) {
        var delta = goal[j] - start[j];
        positions.push(start[j] + delta * alpha);
        velocities.push(delta * sample.velocities[0] / distance);
        accelerations.push(delta * sample.accelerations[0] / distance);
      }
      mapped.push(new JointTrajectorySample(sample.timeSeconds, positions,
        velocities, accelerations));
    }
    activeTrajectory = new JointTrajectory(mapped);
    elapsedSeconds = 0.0;
    return activeTrajectory;
  }

  /** Software homing for the bootstrap: move to each authored home coordinate. */
  public function home(?options:MotionOptions):JointTrajectory {
    return moveAxes([for (axisValue in axes) new AxisTarget(axisValue.id, axisValue.homePosition)], options);
  }

  /**
   * Advances the local deterministic clock and submits one complete position
   * batch. Pass no duration to use the compiled fixed timestep.
   */
  public function update(?dtSeconds:Float = -1.0):Bool {
    if (activeTrajectory == null) return false;
    var dt = dtSeconds < 0.0 ? fixedTimestepSeconds : dtSeconds;
    if (!Math.isFinite(dt) || dt <= 0.0) throw "Motion-system update duration must be finite and positive";
    var trajectory = activeTrajectory;
    var sample = trajectory.sample(elapsedSeconds);
    var targets:Array<JointTarget> = [];
    for (i in 0...sample.positions.length)
      targets.push(JointTarget.position(i, sample.positions[i]));
    robot.submit(RobotCommand.JointTargets(targets, null));
    if (elapsedSeconds >= trajectory.durationSeconds) {
      activeTrajectory = null;
      return false;
    }
    elapsedSeconds = Math.min(trajectory.durationSeconds, elapsedSeconds + dt);
    return true;
  }

  public function stop(?mode:StopMode = StopMode.Normal):Void {
    activeTrajectory = null;
    elapsedSeconds = 0.0;
    robot.stop(mode);
  }

  function requireAxis(id:String):MotionAxis {
    var result = axis(id);
    if (result == null) throw 'Motion system needs a "$id" axis';
    return cast result;
  }

  function resolveLimits(targets:Array<AxisTarget>, options:MotionOptions):MotionLimits {
    var maxVelocity = options.maxVelocity;
    var maxAcceleration = options.maxAcceleration;
    var maxJerk = options.maxJerk;
    for (target in targets) {
      var axisValue = axis(target.axis);
      if (axisValue == null) throw 'Unknown motion axis "${target.axis}"';
      var resolvedAxis:MotionAxis = cast axisValue;
      var axisVelocity = resolvedAxis.maxVelocity;
      var axisAcceleration = resolvedAxis.maxAcceleration;
      if (axisVelocity > 0.0)
        maxVelocity = maxVelocity <= 0.0 ? axisVelocity : Math.min(maxVelocity, axisVelocity);
      if (axisAcceleration > 0.0)
        maxAcceleration = maxAcceleration <= 0.0 ? axisAcceleration : Math.min(maxAcceleration, axisAcceleration);
    }
    return new MotionLimits(maxVelocity, maxAcceleration, maxJerk);
  }
}

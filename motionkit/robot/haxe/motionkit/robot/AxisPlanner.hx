package motionkit.robot;

import motionkit.AxisTarget;
import motionkit.MotionOptions;
import motionkit.axis.MotionAxis;
import motionkit.trajectory.MotionLimits;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.TrajectoryState;

typedef AxisPlanRequest = {
  var targets:Array<AxisTarget>;
  var options:Null<MotionOptions>;
}

/** Joint-axis planner with immutable machine configuration and no request state. */
class AxisPlanner {
  final axes:Array<MotionAxis>;
  final fixedTimestepSeconds:Float;

  public function new(axes:Array<MotionAxis>, fixedTimestepSeconds:Float) {
    this.axes = axes.copy();
    this.fixedTimestepSeconds = fixedTimestepSeconds;
  }

  function axis(id:String):Null<MotionAxis> {
    for (value in axes) if (value.id == id) return value;
    return null;
  }

  public function plan(start:Array<Float>, request:AxisPlanRequest):PlanningResult {
    var targets = request.targets;
    var options = request.options;
    if (targets == null || targets.length == 0) throw "Axis move needs at least one target";
    var movedAxes:Array<MotionAxis> = [];
    var logicalGoal:Array<Float> = [];
    var seen = new Map<String, Bool>();
    for (target in targets) {
      if (target == null) throw "Axis move cannot contain a null target";
      var axisValue = axis(target.axis);
      if (axisValue == null) throw 'Unknown motion axis "${target.axis}"';
      if (seen.exists(target.axis)) throw 'Axis move targets "${target.axis}" more than once';
      if (target.position < axisValue.lowerLimit || target.position > axisValue.upperLimit)
        throw 'Axis "${target.axis}" target ${target.position} is outside its limits';
      movedAxes.push(axisValue);
      logicalGoal.push(target.position);
      seen.set(target.axis, true);
    }
    var chosenOptions = options == null ? new MotionOptions() : options;
    return {trajectory: planLogical(start, movedAxes, logicalGoal,
      resolveLimits(targets, chosenOptions)), report: null, diagnostics: []};
  }

  public function planJog(start:Array<Float>, axisValue:MotionAxis, velocity:Float,
      durationSeconds:Float, acceleration:Float):PlanningResult {
    var startLogical = axisValue.logicalPosition(start);
    var requestedEnd = startLogical + velocity * durationSeconds;
    var endLogical = Math.max(axisValue.lowerLimit,
      Math.min(axisValue.upperLimit, requestedEnd));
    return {trajectory: planLogical(start, [axisValue], [endLogical],
      new MotionLimits(Math.abs(velocity), acceleration)),
      report: null, diagnostics: []};
  }

  public function planContinuedJog(state:TrajectoryState, axisValue:MotionAxis,
      velocity:Float, durationSeconds:Float, acceleration:Float):PlanningResult {
    var target = state.positions.copy();
    var logical = axisValue.logicalPosition(target);
    var end = Math.max(axisValue.lowerLimit,
      Math.min(axisValue.upperLimit, logical + velocity * durationSeconds));
    axisValue.writeLogicalPosition(target, end);
    var currentLogicalVelocity = axisValue.logicalPosition(
      [for (joint in 0...state.velocities.length)
        state.positions[joint] + state.velocities[joint]]) - logical;
    return {trajectory: nativeLogicalPlan(state.positions, state.velocities,
      state.accelerations, target,
      new MotionLimits(Math.max(Math.abs(velocity), Math.abs(currentLogicalVelocity)),
        acceleration)), report: null, diagnostics: []};
  }

  public function planRetarget(state:TrajectoryState, targets:Array<AxisTarget>,
      options:Null<MotionOptions>):PlanningResult {
    var target = state.positions.copy();
    var seen = new Map<String, Bool>();
    for (requested in targets) {
      if (requested == null) throw "Axis move cannot contain a null target";
      var axisValue = axis(requested.axis);
      if (axisValue == null) throw 'Unknown motion axis "${requested.axis}"';
      if (seen.exists(requested.axis))
        throw 'Axis move targets "${requested.axis}" more than once';
      if (requested.position < axisValue.lowerLimit ||
          requested.position > axisValue.upperLimit)
        throw 'Axis "${requested.axis}" target ${requested.position} is outside its limits';
      axisValue.writeLogicalPosition(target, requested.position);
      seen.set(requested.axis, true);
    }
    var limits = resolveLimits(targets,
      options == null ? new MotionOptions() : options);
    return {trajectory: nativeLogicalPlan(state.positions, state.velocities,
      state.accelerations, target, limits), report: null, diagnostics: []};
  }

  /**
   * Plans in logical axis units and maps the native polynomial onto joints. The
   * limits are the axes' authored units, and every joint of an axis follows
   * that axis's one profile, so the motors of a geared or dual-motor axis stay
   * in proportion throughout the move. Joints of other axes keep `start`.
   */
  function planLogical(start:Array<Float>, movedAxes:Array<MotionAxis>, logicalGoal:Array<Float>,
      limits:MotionLimits):Trajectory {
    var target = start.copy();
    for (index in 0...movedAxes.length)
      movedAxes[index].writeLogicalPosition(target, logicalGoal[index]);
    var stationary = true;
    for (joint in 0...start.length)
      if (Math.abs(target[joint] - start[joint]) > 1e-12) stationary = false;
    if (stationary)
      return Trajectory.fromPositionSamples([0.0, fixedTimestepSeconds], [start, start]);
    return nativeLogicalPlan(start, [for (_ in start) 0.0],
      [for (_ in start) 0.0], target, limits);
  }

  function nativeLogicalPlan(start:Array<Float>, velocity:Array<Float>,
      acceleration:Array<Float>, target:Array<Float>, limits:MotionLimits):Trajectory {
    var maxVelocity = [for (_ in start) 0.0];
    var maxAcceleration = [for (_ in start) 0.0];
    var maxJerk = [for (_ in start) 0.0];
    for (axisValue in axes) {
      var scales = [for (_ in start) 0.0];
      axisValue.writeLogicalDelta(scales, 1.0);
      var velocityLimit = limits.maxVelocity > 0.0 ?
        Math.min(axisValue.maxVelocity, limits.maxVelocity) : axisValue.maxVelocity;
      var accelerationLimit = limits.maxAcceleration > 0.0 ?
        Math.min(axisValue.maxAcceleration, limits.maxAcceleration) : axisValue.maxAcceleration;
      var jerkLimit = limits.maxJerk > 0.0 ? limits.maxJerk :
        accelerationLimit / fixedTimestepSeconds;
      for (joint in axisValue.jointIndices) {
        var scale = Math.abs(scales[joint]);
        var speed = velocityLimit * scale;
        var accel = accelerationLimit * scale;
        var jerk = jerkLimit * scale;
        maxVelocity[joint] = maxVelocity[joint] <= 0.0 ? speed :
          Math.min(maxVelocity[joint], speed);
        maxAcceleration[joint] = maxAcceleration[joint] <= 0.0 ? accel :
          Math.min(maxAcceleration[joint], accel);
        maxJerk[joint] = maxJerk[joint] <= 0.0 ? jerk : Math.min(maxJerk[joint], jerk);
      }
    }
    for (joint in 0...start.length)
      if (maxVelocity[joint] <= 0.0 || maxAcceleration[joint] <= 0.0 ||
          maxJerk[joint] <= 0.0)
        throw 'Joint $joint needs positive velocity, acceleration and jerk limits';
    var native = Trajectory.generateStateToState(start, velocity, acceleration, target,
      maxVelocity, maxAcceleration, maxJerk);
    return native;
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

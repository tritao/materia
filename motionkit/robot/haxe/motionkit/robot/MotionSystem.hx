package motionkit.robot;

import haxe.Int64;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.axis.MotionAxis;
import motionkit.path.PathPoint;
import motionkit.path.GeometricPath;
import motionkit.planner.PathPlanningOptions;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.TrajectoryState;
import motionkit.trajectory.ValidationReport;
import MotionKitNative;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.StopMode;
import RobotKitRuntime;

/**
 * Semantic machine-axis view over an ordinary RobotKit robot.
 *
 * The view owns planning and buffered-execution state, while the Robot still
 * owns snapshots, transport, safety, and lifecycle. Immediate and queued
 * motions use the same RobotCommand boundary.
 *
 * Every change of speed stays within the joints' acceleration limits: a hold
 * slows along the path, a resume speeds back up along it, and a new immediate
 * move while moving first brings the machine to rest along its current path,
 * then plans from where it stopped.
 */
class MotionSystem {
  public final robot:Robot;
  public final axes:Array<MotionAxis>;
  final axisPlanner:AxisPlanner;
  final pathPlanner:PathPlanner;
  public final fixedTimestepSeconds:Float;
  public final replacementMarginOwnerPeriods:Int;
  public final replacementOwnerPeriodSeconds:Float;
  /** Joint and sampled Cartesian checks for the last planned path. */
  public var lastPathValidationReport(default, null):Null<ValidationReport> = null;
  var pathJerkUnchecked:haxe.ds.ObjectMap<Trajectory, Bool> = new haxe.ds.ObjectMap();
  public var lastPathPlanningDiagnostics(default, null):Array<String> = [];
  /** Native trajectory currently submitted to the runtime queue. */
  var activeTrajectory:Null<Trajectory> = null;
  var queuedTrajectories:Array<Trajectory> = [];
  final stream:TrajectoryStream;
  var activeStationary:Bool = false;
  var elapsedSeconds(get, never):Float;
  var held:Bool = false;
  /** A native lifecycle command must reach the owner before another plan. */
  var nativeRefillDeferred:Bool = false;
  var bufferedTotalSeconds:Float = 0.0;
  var bufferedCompletedSeconds:Float = 0.0;
  var plannedEndPositions:Null<Array<Float>> = null;
  /** True while stopping so that deferred work can start from rest. */
  var stoppingForReplacement:Bool = false;
  /** Work deferred until a replacement stop reaches rest, in order. */
  var afterStop:Array<Void -> Void> = [];
  /** Logical axis of the active jog, which a further jog can change without stopping. */
  var activeJogAxis:Null<String> = null;
  final modelRevision:Int64;
  final calibrationRevision:Int64;

  public static function fromBlueprint(robot:Robot, blueprint:MotionSystemBlueprint):MotionSystem
    return new MotionSystem(robot, blueprint);

  public function new(robot:Robot, blueprint:MotionSystemBlueprint) {
    if (robot == null || blueprint == null) throw "Motion system needs a robot and blueprint";
    if (!robot.capabilities().supportsTrajectoryQueue ||
        !robot.capabilities().supportsExecutionPlans)
      throw "MotionSystem requires a robot with trajectory queue and execution plan support";
    var description = robot.description();
    if (description == null || description.joints.length != blueprint.model.joints.length)
      throw "Robot description does not match the motion-system model";
    if (description.joints.length > RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_JOINTS)
      throw "MotionSystem robot exceeds the execution plan joint limit";
    for (i in 0...blueprint.model.joints.length) {
      if (description.joints[i] != blueprint.model.joints[i].name)
        throw 'Robot joint $i does not match motion-system joint "${blueprint.model.joints[i].name}"';
    }
    this.robot = robot;
    stream = new TrajectoryStream(robot);
    this.modelRevision = Int64.ofInt(blueprint.runtime.revision);
    this.calibrationRevision = Int64.ofInt(blueprint.runtime.calibrationRevision);
    this.fixedTimestepSeconds = blueprint.fixedTimestepSeconds;
    if (blueprint.replacementMarginOwnerPeriods < 1)
      throw "Smooth replacement margin must be at least one owner period";
    if (!Math.isFinite(blueprint.replacementOwnerPeriodSeconds) ||
        blueprint.replacementOwnerPeriodSeconds <= 0.0)
      throw "Smooth replacement owner period must be finite and positive";
    this.replacementMarginOwnerPeriods = blueprint.replacementMarginOwnerPeriods;
    this.replacementOwnerPeriodSeconds = blueprint.replacementOwnerPeriodSeconds;
    this.axes = [];
    var axisIds = new Map<String, Bool>();
    for (axisBlueprint in blueprint.axes) {
      var axis = new MotionAxis(axisBlueprint, description.joints);
      if (axisIds.exists(axis.id)) throw 'Duplicate motion axis "${axis.id}"';
      axisIds.set(axis.id, true);
      this.axes.push(axis);
    }
    axisPlanner = new AxisPlanner(this.axes, fixedTimestepSeconds);
    pathPlanner = new PathPlanner(this.axes, fixedTimestepSeconds,
      modelRevision, calibrationRevision);
  }

  public function axis(id:String):Null<MotionAxis> {
    for (value in axes) if (value.id == id) return value;
    return null;
  }

  /** True while a trajectory is active, held, or waiting to start after a stop. */
  public function isMoving():Bool return activeTrajectory != null || afterStop.length > 0;

  public function trajectory():Null<Trajectory> return activeTrajectory;

  /** Number of active and waiting trajectories currently owned by the system. */
  public function queueDepth():Int
    return (activeTrajectory == null ? 0 : 1) + queuedTrajectories.length;

  /** Remaining duration across the active trajectory and all queued trajectories. */
  public function queuedDurationSeconds():Float {
    var result = activeTrajectory == null ? 0.0 :
      Math.max(0.0, activeTrajectory.durationSeconds() - elapsedSeconds);
    for (trajectoryValue in queuedTrajectories) result += trajectoryValue.durationSeconds();
    return result;
  }

  /** Completion fraction of the currently buffered motion, in the range [0, 1]. */
  public function progress():Float {
    if (activeTrajectory != null) syncFromRuntime();
    if (bufferedTotalSeconds <= 0.0) return 1.0;
    var source = activeTrajectory;
    var completed = bufferedCompletedSeconds + (source == null ? 0.0
      : Math.min(source.durationSeconds(), elapsedSeconds));
    return Math.min(1.0, Math.max(0.0, completed / bufferedTotalSeconds));
  }

  public function isHolding():Bool return held;

  /**
   * Plans a coordinated move while leaving unspecified axes at their current
   * positions, replacing any buffered motion. While the machine is moving it
   * first stops along its current path; the move is then planned from where
   * it came to rest and null is returned, as the plan does not exist yet.
   */
  public function moveAxes(targets:Array<AxisTarget>, ?options:MotionOptions):Null<Trajectory> {
    var retargeted = retargetNativeAxes(targets, options);
    if (retargeted != null) return retargeted;
    var planned = planAxesFrom(robot.snapshot().positions.toArray(), targets, options);
    return replaceMotion(planned,
      () -> planAxesFrom(robot.snapshot().positions.toArray(), targets, options));
  }

  /** Adds a coordinated axis move behind all motion already in the buffer. */
  public function queueAxes(targets:Array<AxisTarget>, ?options:MotionOptions):Null<Trajectory> {
    if (afterStop.length > 0) {
      var captured = targets == null ? null : targets.copy();
      afterStop.push(() -> queueAxes(captured, options));
      return null;
    }
    var trajectoryValue = planAxesFrom(planningStartPositions(), targets, options);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds an already planned joint trajectory to the execution buffer. */
  public function queueTrajectory(trajectoryValue:Trajectory):Void {
    if (trajectoryValue == null) throw "Queued trajectory is required";
    var jointCount = robot.description().joints.length;
    if (trajectoryValue.jointCount() != jointCount)
      throw 'Queued trajectory has ${trajectoryValue.jointCount()} joints; robot has $jointCount';
    if (afterStop.length > 0) {
      afterStop.push(() -> queueTrajectory(trajectoryValue));
      return;
    }
    enqueueTrajectory(trajectoryValue);
  }

  /**
   * Plans a straight Cartesian move for a direct XYZ gantry. Like moveAxes,
   * it first stops along the current path when the machine is moving.
   */
  public function moveLinear(target:PathPoint, feed:Feed,
      ?options:MotionOptions):Null<Trajectory> {
    var planned = planLinearPathFrom(robot.snapshot().positions.toArray(), target, feed, options);
    return replaceMotion(planned,
      () -> planLinearPathFrom(robot.snapshot().positions.toArray(), target, feed, options));
  }

  /** Adds a straight Cartesian move behind all motion already in the buffer. */
  public function queueLinear(target:PathPoint, feed:Feed,
      ?options:MotionOptions):Null<Trajectory> {
    if (afterStop.length > 0) {
      var captured = target == null ? null : new PathPoint(target.x, target.y, target.z);
      afterStop.push(() -> queueLinear(captured, feed, options));
      return null;
    }
    var trajectoryValue = planLinearPathFrom(planningStartPositions(), target, feed, options);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /**
   * Plans a connected Cartesian polyline for a direct XYZ machine. The path
   * must start where the machine is, so it cannot replace motion in progress:
   * hold and wait for rest first.
   */
  public function movePath(path:GeometricPath, ?pathOptions:PathPlanningOptions,
      ?motionOptions:MotionOptions):Trajectory {
    if (isMotionInProgress())
      throw "A path must start where the machine is at rest; hold and wait before replacing motion with a path";
    var trajectoryValue = planPathFrom(robot.snapshot().positions.toArray(), path,
      pathOptions, motionOptions);
    clearBufferedMotion();
    beginImmediate(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds a connected Cartesian polyline behind motion already in the buffer. */
  public function queuePath(path:GeometricPath, ?pathOptions:PathPlanningOptions,
      ?motionOptions:MotionOptions):Null<Trajectory> {
    if (afterStop.length > 0) {
      var captured = path == null ? null : new GeometricPath(path.primitives);
      afterStop.push(() -> queuePath(captured, pathOptions, motionOptions));
      return null;
    }
    var trajectoryValue = planPathFrom(planningStartPositions(), path,
      pathOptions, motionOptions);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Holds buffered motion and slows to rest along the path without discarding it. */
  public function hold():Void {
    if (held) return;
    held = true;
    if (activeTrajectory != null) robot.submit(RobotCommand.Hold);
  }

  /**
   * Resumes a held buffer from where it stopped, speeding back up along the
   * path. If the hold is still slowing down, it resumes once at rest.
   */
  public function resume():Void {
    if (!held) return;
    if (activeTrajectory != null) {
      if (trajectoryFinishedInRuntime(syncFromRuntime())) {
        completeActiveTrajectory();
        held = false;
        activateNextTrajectory();
        return;
      }
      robot.submit(RobotCommand.Resume);
      nativeRefillDeferred = true;
      held = false;
      return;
    }
    held = false;
    activateNextTrajectory();
  }

  /**
   * Aborts buffered motion. A normal abort slows to rest along the current
   * path before discarding it; an emergency stop acts immediately.
   */
  public function abort(?mode:StopMode = StopMode.Normal):Void {
    if (mode == StopMode.Normal && isMotionInProgress() && elapsedSeconds > 1e-9) {
      robot.submit(RobotCommand.Abort);
      queuedTrajectories = [];
      afterStop = [];
      held = false;
      stoppingForReplacement = true;
      return;
    }
    clearBufferedMotion();
    robot.stop(mode);
  }

  /**
   * Starts `planned` now when the machine is at rest. Otherwise stops along
   * the current path and starts a fresh `replan` from where it came to rest.
   */
  function replaceMotion(planned:Trajectory, replan:Void -> Trajectory,
      ?jogAxis:String):Null<Trajectory> {
    if (!isMotionInProgress()) {
      clearBufferedMotion();
      beginImmediate(planned);
      activeJogAxis = jogAxis;
      return planned;
    }
    discardNativeTrajectory(planned);
    queuedTrajectories = [];
    afterStop = [() -> {
      clearBufferedMotion();
      beginImmediate(replan());
      activeJogAxis = jogAxis;
    }];
    held = false;
    stoppingForReplacement = true;
    beginStop();
    return null;
  }

  /**
   * Whether the machine is in motion or may still be: a trajectory that has
   * started and not finished, or a stop still slowing down.
   */
  function isMotionInProgress():Bool {
    if (stoppingForReplacement) return true;
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return false;
    if (syncFromRuntime().trajectoryActive) return true;
    return elapsedSeconds > 1e-9 && elapsedSeconds < trajectoryValue.durationSeconds() - 1e-9 &&
      !(held && stopSettled());
  }

  /** Slows the active trajectory to rest along its path. */
  function beginStop():Void {
    if (activeTrajectory != null) {
      syncFromRuntime();
      robot.stop(StopMode.Normal);
    }
  }

  /** True once a requested stop has reached rest. */
  function stopSettled():Bool {
    var observation = syncFromRuntime();
    return !observation.trajectoryActive &&
      observation.sessionState != RobotKitRuntimeConstants.RK_SESSION_STOPPING;
  }

  /** Runs deferred work or a pending resume once a stop has reached rest. */
  function advanceStop():Void {
    if (!stopSettled()) return;
    if (stoppingForReplacement && !held) {
      stoppingForReplacement = false;
      held = false;
      var actions = afterStop;
      afterStop = [];
      for (action in actions) action();
      return;
    }
  }

  function planLinearPathFrom(start:Array<Float>, target:PathPoint, feed:Feed,
      options:Null<MotionOptions>):Trajectory {
    lastPathValidationReport = null;
    lastPathPlanningDiagnostics = [];
    return storePathPlan(pathPlanner.planLinear(start,
      {target: target, feed: feed, options: options}));
  }

  function planPathFrom(start:Array<Float>, path:GeometricPath,
      pathOptions:Null<PathPlanningOptions>, motionOptions:Null<MotionOptions>):Trajectory {
    lastPathValidationReport = null;
    lastPathPlanningDiagnostics = [];
    return storePathPlan(pathPlanner.plan(start,
      {path: path, pathOptions: pathOptions, motionOptions: motionOptions}));
  }

  function storePathPlan(result:PlanningResult):Trajectory {
    lastPathValidationReport = result.report;
    lastPathPlanningDiagnostics = result.diagnostics;
    if (result.report != null)
      pathJerkUnchecked.set(result.trajectory,
        result.report.checks[MotionKitNativeConstants.MK_CHECK_JERK].status ==
        MotionKitNativeConstants.MK_CHECK_UNCHECKED);
    return result.trajectory;
  }

  /** Software homing for the bootstrap: move to each authored home coordinate. */
  public function home(?options:MotionOptions):Null<Trajectory> {
    return moveAxes([for (axisValue in axes) new AxisTarget(axisValue.id, axisValue.homePosition)], options);
  }

  /**
   * Plans a bounded timed jog in one logical axis. The endpoint is clamped to
   * the authored axis limits, while all physical joints in a coordinated axis
   * group receive the same logical displacement and scaled velocity.
   */
  public function jog(axisId:String, velocity:Float, durationSeconds:Float,
      ?maxAcceleration:Float):Null<Trajectory> {
    var axisValue = axis(axisId);
    if (axisValue == null) throw 'Unknown motion axis "$axisId"';
    if (!Math.isFinite(velocity) || velocity == 0.0)
      throw "Jog velocity must be finite and non-zero";
    if (!Math.isFinite(durationSeconds) || durationSeconds <= 0.0)
      throw "Jog duration must be finite and positive";
    if (axisValue.maxVelocity > 0.0 && Math.abs(velocity) > axisValue.maxVelocity + 1e-12)
      throw 'Jog velocity $velocity exceeds axis "$axisId" maximum ${axisValue.maxVelocity}';
    var acceleration = maxAcceleration == null ? axisValue.maxAcceleration : maxAcceleration;
    if (!Math.isFinite(acceleration) || acceleration <= 0.0)
      throw 'Jog axis "$axisId" needs a positive acceleration limit';

    var continued = continueJog(axisValue, velocity, durationSeconds, acceleration);
    if (continued != null) return continued;
    function planJog():Trajectory {
      var start = robot.snapshot().positions.toArray();
      return axisPlanner.planJog(start, axisValue, velocity,
        durationSeconds, acceleration).trajectory;
    }
    return replaceMotion(planJog(), planJog, axisValue.id);
  }

  /**
   * Changes a running jog on the same axis at the committed plan state.
   */
  function continueJog(axisValue:MotionAxis, velocity:Float, durationSeconds:Float,
      acceleration:Float):Null<Trajectory> {
    var executing = activeTrajectory;
    if (executing == null || activeJogAxis != axisValue.id || held || stoppingForReplacement ||
        afterStop.length > 0 || queuedTrajectories.length > 0)
      return null;
    var smooth = executing;
    if (smooth != null) {
      return trySmoothReplacement(executing, axisValue.id, state ->
        axisPlanner.planContinuedJog(state, axisValue, velocity,
          durationSeconds, acceleration).trajectory);
    }
    return null;
  }

  function retargetNativeAxes(targets:Array<AxisTarget>,
      options:Null<MotionOptions>):Null<Trajectory> {
    var executing = activeTrajectory;
    if (executing == null || held || stoppingForReplacement ||
        afterStop.length > 0 || queuedTrajectories.length > 0)
      return null;
    var smooth = executing;
    if (smooth == null || smooth.segments()[0].coefficients[0].length < 3) return null;
    return trySmoothReplacement(executing, null, state ->
      axisPlanner.planRetarget(state, targets, options).trajectory);
  }

  function trySmoothReplacement(executing:Trajectory, jogAxis:Null<String>,
      planFromState:TrajectoryState -> Trajectory):Null<Trajectory> {
    if (executing == null) return null;
    return TrajectoryStream.replaceWithRetry(executing, syncFromRuntime,
      replacementOwnerPeriodSeconds, replacementMarginOwnerPeriods, planFromState,
      (planned, state, observation, anchorNs) ->
        submitSmoothReplacement(planned, state, observation, anchorNs, jogAxis));
  }

  function submitSmoothReplacement(planned:Trajectory, state:TrajectoryState,
      observation:RobotSnapshot, anchorNs:Int64, jogAxis:Null<String>):Trajectory {
    var tag = stream.submitSmoothReplacement(planned, state, observation,
      anchorNs, modelRevision, calibrationRevision);
    setActive(planned);
    activeJogAxis = jogAxis;
    bufferedTotalSeconds = planned.durationSeconds();
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(planned);
    stream.recordReplacement(tag);
    return planned;
  }

  /**
   * Advances the local deterministic clock and submits one complete position
   * batch. Pass no duration to use the compiled fixed timestep.
   */
  public function update(?dtSeconds:Float = -1.0):Bool {
    var dt = dtSeconds < 0.0 ? fixedTimestepSeconds : dtSeconds;
    if (!Math.isFinite(dt) || dt <= 0.0) throw "Motion-system update duration must be finite and positive";
    if (held || stoppingForReplacement) {
      // No refills while stopping: beginStop() already queued enough path for
      // the whole stop, and a late chunk could otherwise land after the stop
      // finished and be taken as a resume.
      var stopping = activeTrajectory;
      if (stopping != null && trajectoryFinishedInRuntime(syncFromRuntime()))
        completeActiveTrajectory();
      advanceStop();
      if (held || stoppingForReplacement) return false;
    }
    if (activeTrajectory == null) activateNextTrajectory();
    if (activeTrajectory == null) return false;
    var trajectory = activeTrajectory;
    if (trajectory.durationSeconds() <= 0.0 || activeStationary) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    var observation = syncFromRuntime();
    if (stoppingForReplacement) return true;
    if (trajectoryFinishedInRuntime(observation)) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    if (nativeRefillDeferred) nativeRefillDeferred = false;
    else if (!stream.submitted || stream.shouldRefill(dt, fixedTimestepSeconds))
      fillNativeWindow();
    return true;
  }

  public function stop(?mode:StopMode = StopMode.Normal):Void {
    abort(mode);
  }

  /** Starts executing `trajectoryValue` as a fresh, un-retimed plan. */
  function setActive(trajectoryValue:Trajectory):Void {
    activeTrajectory = trajectoryValue;
    var segments = trajectoryValue.segments();
    activeStationary = stationarySegments(segments);
    activeJogAxis = null;
    stream.begin(segments, trajectoryValue.durationSeconds());
  }

  function planAxesFrom(start:Array<Float>, targets:Array<AxisTarget>,
      options:Null<MotionOptions>):Trajectory
    return axisPlanner.plan(start, {targets: targets, options: options}).trajectory;

  function planningStartPositions():Array<Float> {
    if (plannedEndPositions != null) return plannedEndPositions.copy();
    return robot.snapshot().positions.toArray();
  }

  function beginImmediate(trajectoryValue:Trajectory):Void {
    setActive(trajectoryValue);
    bufferedTotalSeconds = trajectoryValue.durationSeconds();
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(trajectoryValue);
    fillNativeWindow();
  }

  function enqueueTrajectory(trajectoryValue:Trajectory):Void {
    if (activeTrajectory == null && queuedTrajectories.length == 0) {
      bufferedTotalSeconds = 0.0;
      bufferedCompletedSeconds = 0.0;
    }
    queuedTrajectories.push(trajectoryValue);
    bufferedTotalSeconds += trajectoryValue.durationSeconds();
    plannedEndPositions = trajectoryEnd(trajectoryValue);
    activateNextTrajectory();
  }

  function activateNextTrajectory():Void {
    if (held || stoppingForReplacement || activeTrajectory != null ||
        queuedTrajectories.length == 0)
      return;
    var next:Trajectory = queuedTrajectories.shift();
    setActive(next);
    fillNativeWindow();
  }

  function clearBufferedMotion():Void {
    activeTrajectory = null;
    activeStationary = false;
    queuedTrajectories = [];
    held = false;
    stoppingForReplacement = false;
    afterStop = [];
    activeJogAxis = null;
    bufferedTotalSeconds = 0.0;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = null;
    nativeRefillDeferred = false;
    stream.clear();
  }

  /** Keep two seconds of motion queued so HOLD can slow along the path. */
  function fillNativeWindow():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    var jerkUnchecked = pathJerkUnchecked.exists(trajectoryValue) &&
      pathJerkUnchecked.get(trajectoryValue) == true;
    stream.fill(robot.snapshot().positions.length, false,
      (first, last, tag, startNs, _) -> stream.motionSubmission(
        trajectoryValue, first, last, tag, startNs, modelRevision,
        calibrationRevision, jerkUnchecked),
      (first, last, error) ->
        'plan chunk [$first,$last] of ${trajectoryValue.segments().length}: $error');
  }

  function get_elapsedSeconds():Float return stream.elapsedSeconds;

  function syncFromRuntime():RobotSnapshot
    return activeTrajectory == null ? robot.snapshot() : stream.sync();

  function trajectoryFinishedInRuntime(observation:RobotSnapshot):Bool
    return activeTrajectory != null && stream.finishedMotion(observation);

  function completeActiveTrajectory():Void {
    if (activeTrajectory == null) return;
    bufferedCompletedSeconds += activeTrajectory.durationSeconds();
    pathJerkUnchecked.remove(activeTrajectory);
    activeTrajectory = null;
    activeStationary = false;
    stream.clear();
    activateNextTrajectory();
  }

  function discardNativeTrajectory(value:Trajectory):Void {
    pathJerkUnchecked.remove(value);
    value.dispose();
  }

  static function trajectoryEnd(trajectoryValue:Trajectory):Array<Float>
    return trajectoryValue.evaluate(trajectoryValue.durationSeconds()).positions.copy();

  static function stationarySegments(segments:Array<{timeFromStartNs:Int64,
      durationNs:Int64, coefficients:Array<Array<Float>>}>):Bool {
    for (segment in segments)
      for (coefficients in segment.coefficients)
        for (index in 1...coefficients.length)
          if (Math.abs(coefficients[index]) > 1e-15) return false;
    return true;
  }


}

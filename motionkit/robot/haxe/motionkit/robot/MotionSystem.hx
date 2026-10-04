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
import TrajectoryCore;
import robotkit.core.Robot;
import robotkit.core.RobotCommand;
import robotkit.core.RobotSnapshot;
import robotkit.core.StopMode;
import robotkit.runtime.RobotRuntimeError;
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
  /** Drive-check settings shared by immediate, queued, path and replacement motion. */
  public var planCheck(get, set):Null<PlanCheck>;
  public var planChecks(get, never):PlanCheckSummary;
  function get_planCheck():Null<PlanCheck> return stream.planCheck;
  function set_planCheck(value:Null<PlanCheck>):Null<PlanCheck> return stream.planCheck = value;
  function get_planChecks():PlanCheckSummary return stream.planChecks;
  public final axes:Array<MotionAxis>;
  final axisPlanner:AxisPlanner;
  final homingAxes:Array<HomingAxis> = [];
  var homingDriver:Null<HomingDriver> = null;
  var homingSides:Null<robotkit.runtime.HomingSideControl> = null;
  var homingCycle:Null<HomingCycle> = null;
  final pathPlanner:PathPlanner;
  public final fixedTimestepSeconds:Float;
  public final replacementMarginOwnerPeriods:Int;
  public final replacementOwnerPeriodSeconds:Float;
  /** Joint and sampled Cartesian checks for the last planned path. */
  public var lastPathValidationReport(default, null):Null<ValidationReport> = null;
  var jogTrajectories:haxe.ds.ObjectMap<Trajectory, Bool> = new haxe.ds.ObjectMap();
  var pathJerkUnchecked:haxe.ds.ObjectMap<Trajectory, Bool> = new haxe.ds.ObjectMap();
  public var lastPathPlanningDiagnostics(default, null):Array<String> = [];
  /** Native trajectory currently submitted to the runtime queue. */
  var activeTrajectory:Null<Trajectory> = null;
  var queuedTrajectories:Array<Trajectory> = [];
  final stream:TrajectoryStream;
  final jointTolerances:Array<Float>;
  final session:MotionSession = new MotionSession();
  var activeStationary:Bool = false;
  var elapsedSeconds(get, never):Float;
  /** A native lifecycle command must reach the owner before another plan. */
  var nativeRefillDeferred:Bool = false;
  /** Updates so far, and the one that last submitted motion: an update submits once. */
  var updateCount:Int = 0;
  var filledAtUpdate:Int = -1;
  var bufferedTotalSeconds:Float = 0.0;
  var bufferedCompletedSeconds:Float = 0.0;
  var plannedEndPositions:Null<Array<Float>> = null;
  final modelRevision:Int64;
  final calibrationRevision:Int64;

  public static function fromBlueprint(robot:Robot, blueprint:MotionSystemBlueprint):MotionSystem
    return new MotionSystem(robot, blueprint);

  public function new(robot:Robot, blueprint:MotionSystemBlueprint) {
    if (robot == null || blueprint == null) throw "Motion system needs a robot and blueprint";
    if (!robot.capabilities().execution.plans)
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
    stream.planCheck = new PlanCheck(blueprint.model, [for (joint in blueprint.model.joints) joint.id]);
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
    jointTolerances = [for (_ in description.joints) 1e-6];
    var axisIds = new Map<String, Bool>();
    var hasHomes = false;
    for (contact in blueprint.runtime.switches) if (contact.role == "home") hasHomes = true;
    var resolvedAxes = hasHomes
      ? MotionAxisCouplings.expand(blueprint.axes, blueprint.runtime, description.joints) : blueprint.axes;
    for (axisBlueprint in resolvedAxes) {
      var axis = new MotionAxis(axisBlueprint, description.joints);
      if (axisIds.exists(axis.id)) throw 'Duplicate motion axis "${axis.id}"';
      axisIds.set(axis.id, true);
      this.axes.push(axis);
      var tolerance = [for (_ in description.joints) 0.0];
      axis.writeLogicalDelta(tolerance, 1e-6);
      for (joint in axis.jointIndices) jointTolerances[joint] = Math.abs(tolerance[joint]);
    }
    for (axis in this.axes) {
      var joint = axis.jointIndices[0];
      var homes = [for (contact in blueprint.runtime.switches)
        if (contact.role == "home" && contact.joint == description.joints[joint]) contact];
      if (homes.length > 0) {
        var physical = blueprint.runtime.joints[joint];
        if (physical.maxRate == null || physical.maxAcceleration == null)
          throw 'Homing axis "${axis.id}" needs physical drive limits';
        homingAxes.push(new HomingAxis(axis.id, joint, homes, physical.maxRate,
          physical.maxAcceleration, physical.lowerLimit, physical.upperLimit,
          physical.overtravel, axis.jointOffset(0) + axis.jointScale(0) * axis.homePosition,
          fixedTimestepSeconds));
      }
    }
    for (contact in blueprint.runtime.switches) if (contact.role == "home") {
      var mapped = false;
      for (home in homingAxes) for (signal in home.switches) if (signal.id == contact.id) mapped = true;
      if (!mapped) throw 'Home switch "${contact.id}" has no independent motion axis';
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
  public function isMoving():Bool
    return (homingCycle != null && homingCycle.isActive()) ||
      activeTrajectory != null || session.isStopping() || session.hasPending();

  public function sessionState():SessionState return session.state;

  /** A fault is latched until the application explicitly resets runtime safety. */
  public function reset():Void {
    if (!session.isFaulted()) return;
    robot.resetSafety();
    var snapshot = robot.snapshot();
    if (snapshot.faultCode != 0) {
      session.observe(snapshot);
      throw 'Motion runtime fault ${snapshot.faultCode} remains after reset';
    }
    clearBufferedMotionAfterFault();
    session.reset();
  }

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

  public function isHolding():Bool return session.isHolding();

  /**
   * Plans a coordinated move while leaving unspecified axes at their current
   * positions, replacing any buffered motion. While the machine is moving it
   * first stops along its current path; the move is then planned from where
   * it came to rest and null is returned, as the plan does not exist yet.
   */
  public function moveAxes(targets:Array<AxisTarget>, ?options:MotionOptions):Null<Trajectory> {
    checkSnapshot();
    var retargeted = retargetNativeAxes(targets, options);
    if (retargeted != null) return retargeted;
    var planned = planAxesFrom(robot.snapshot().positions.toArray(), targets, options);
    return replaceMotion(planned, MotionRequestCapture.axes(targets, options));
  }

  /** Adds a coordinated axis move behind all motion already in the buffer. */
  public function queueAxes(targets:Array<AxisTarget>, ?options:MotionOptions):Null<Trajectory> {
    checkSnapshot();
    if (session.hasPending()) {
      session.append(MotionRequestCapture.queuedAxes(targets, options));
      return null;
    }
    var trajectoryValue = planAxesFrom(planningStartPositions(), targets, options);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds an already planned joint trajectory to the execution buffer. */
  public function queueTrajectory(trajectoryValue:Trajectory):Void {
    checkSnapshot();
    if (trajectoryValue == null) throw "Queued trajectory is required";
    var jointCount = robot.description().joints.length;
    if (trajectoryValue.jointCount() != jointCount)
      throw 'Queued trajectory has ${trajectoryValue.jointCount()} joints; robot has $jointCount';
    if (session.hasPending()) {
      session.append(MotionRequestCapture.queuedTrajectory(trajectoryValue));
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
    checkSnapshot();
    var planned = planLinearPathFrom(robot.snapshot().positions.toArray(), target, feed, options);
    return replaceMotion(planned, MotionRequestCapture.linear(target, feed, options));
  }

  /** Adds a straight Cartesian move behind all motion already in the buffer. */
  public function queueLinear(target:PathPoint, feed:Feed,
      ?options:MotionOptions):Null<Trajectory> {
    checkSnapshot();
    if (session.hasPending()) {
      session.append(MotionRequestCapture.queuedLinear(target, feed, options));
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
    checkSnapshot();
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
    checkSnapshot();
    if (session.hasPending()) {
      session.append(MotionRequestCapture.queuedPath(path, pathOptions, motionOptions));
      return null;
    }
    var trajectoryValue = planPathFrom(planningStartPositions(), path,
      pathOptions, motionOptions);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Holds buffered motion and slows to rest along the path without discarding it. */
  public function hold():Void {
    checkSnapshot();
    if (session.hold() && activeTrajectory != null) submitCommand(RobotCommand.Hold);
  }

  /**
   * Resumes a held buffer from where it stopped, speeding back up along the
   * path. If the hold is still slowing down, it resumes once at rest.
   */
  public function resume():Void {
    checkSnapshot();
    if (!session.resume()) return;
    if (activeTrajectory != null) {
      submitCommand(RobotCommand.Resume);
      nativeRefillDeferred = true;
    } else activateNextTrajectory();
  }

  /**
   * Aborts buffered motion. A normal abort slows to rest along the current
   * path before discarding it; an emergency stop acts immediately.
   */
  public function abort(?mode:StopMode = StopMode.Normal):Void {
    if (homingCycle != null && homingCycle.isActive()) {
      homingCycle.cancel();
      if (mode == StopMode.Emergency) robot.stop(mode);
      session.stop(Discard, []);
      return;
    }
    checkSnapshot();
    if (mode == StopMode.Normal && isMotionInProgress() && elapsedSeconds > 1e-9) {
      submitCommand(RobotCommand.Abort);
      queuedTrajectories = [];
      session.stop(Discard, []);
      return;
    }
    if (mode != StopMode.Normal && (session.state == Holding || session.isStopping())) {
      stopRobot(mode);
      queuedTrajectories = [];
      session.stop(Discard, []);
      return;
    }
    clearBufferedMotion();
    stopRobot(mode);
  }

  /**
   * Starts `planned` now when the machine is at rest. Otherwise stops along
   * the current path and plans the captured request from where it came to rest.
   */
  function replaceMotion(planned:Trajectory, request:MotionRequest):Null<Trajectory> {
    if (homingCycle != null && homingCycle.isActive()) throw "Ordinary motion cannot interrupt homing";
    if (!isMotionInProgress()) {
      clearBufferedMotion();
      beginImmediate(planned);
      session.jogAxis = switch request {
        case Jog(axisId, _, _, _): axisId;
        case _: null;
      };
      return planned;
    }
    discardNativeTrajectory(planned);
    queuedTrajectories = [];
    session.stop(Replan, [request]);
    beginStop();
    return null;
  }

  /**
   * Whether the machine is in motion or may still be: a trajectory that has
   * started and not finished, or a stop still slowing down.
   */
  function isMotionInProgress():Bool {
    if (session.isStopping() || session.state == Holding) return true;
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return false;
    if (syncFromRuntime().trajectoryActive) return true;
    return elapsedSeconds > 1e-9 && elapsedSeconds < trajectoryValue.durationSeconds() - 1e-9 &&
      !(session.isHolding() && stopSettled());
  }

  /** Slows the active trajectory to rest along its path. */
  function beginStop():Void {
    if (activeTrajectory != null) {
      syncFromRuntime();
      stopRobot(StopMode.Normal);
    }
  }

  /** True once a requested stop has reached rest. */
  function stopSettled():Bool {
    var observation = syncFromRuntime();
    return TrajectoryStream.atRest(observation);
  }

  /** Runs deferred work or a pending resume once a stop has reached rest. */
  function advanceStop():Void {
    if (!stopSettled()) return;
    var wasHolding = session.state == Holding;
    var discarding = session.state == Stopping(Discard);
    var actions = session.rest();
    if (discarding) clearBufferedMotion();
    if (wasHolding && session.resumePending) resume();
    for (action in actions) executeAfterStop(action);
  }

  function executeAfterStop(request:MotionRequest):Void {
    switch request {
      case Axes(targets, options):
        clearBufferedMotion();
        beginImmediate(planAxesFrom(robot.snapshot().positions.toArray(),
          targets, options));
      case Linear(target, feed, options):
        clearBufferedMotion();
        beginImmediate(planLinearPathFrom(robot.snapshot().positions.toArray(),
          target, feed, options));
      case Path(path, pathOptions, motionOptions):
        clearBufferedMotion();
        beginImmediate(planPathFrom(robot.snapshot().positions.toArray(),
          path, pathOptions, motionOptions));
      case Jog(axisId, velocity, durationSeconds, acceleration):
        clearBufferedMotion();
        var axisValue = axis(axisId);
        if (axisValue == null) throw 'Unknown motion axis "$axisId"';
        var plannedJog = axisPlanner.planJog(robot.snapshot().positions.toArray(),
          axisValue, velocity, durationSeconds, acceleration).trajectory;
        jogTrajectories.set(plannedJog, true);
        beginImmediate(plannedJog);
        session.jogAxis = axisId;
      case Queued(command):
        switch command {
          case Axes(targets, options): queueAxes(targets, options);
          case Linear(target, feed, options): queueLinear(target, feed, options);
          case Path(path, pathOptions, motionOptions):
            queuePath(path, pathOptions, motionOptions);
          case Trajectory(segments):
            queueTrajectory(Trajectory.fromSegments(segments));
        }
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
        result.report.checks[TrajectoryCoreConstants.MK_CHECK_JERK].status ==
        TrajectoryCoreConstants.MK_CHECK_UNCHECKED);
    return result.trajectory;
  }

  /** Bind the runtime owner and its encoder/slip reset before sensor homing. */
  public function configureRuntimeHoming(runtime:robotkit.runtime.RobotRuntime,
      afterLatch:Void -> Void, ?sides:robotkit.runtime.HomingSideControl):Void {
    if (isMoving()) throw "Cannot replace a homing driver during motion";
    if (homingAxes.length == 0) throw "Machine has no physical home switches";
    var scopedStop:Null<Void -> Bool> = null;
    if (sides != null && Std.isOfType(sides, robotkit.runtime.HomingStopControl)) {
      var stopControl:robotkit.runtime.HomingStopControl = cast sides;
      scopedStop = () -> stopControl.controlledStop();
    }
    homingDriver = new RuntimeHomingDriver(robot, runtime, homingAxes, axes, afterLatch, scopedStop);
    homingSides = sides;
  }

  public function homingStatus():String
    return homingCycle == null ? "Idle" : homingCycle.status();

  /** Sensor homing when authored switches exist; coordinate move for unswitched robots. */
  public function home(?options:MotionOptions):Null<Trajectory> {
    if (homingAxes.length > 0) {
      if (isMoving() || queuedTrajectories.length > 0) throw "Homing requires an idle motion system";
      var driver = homingDriver;
      if (driver == null) throw "Physical homing requires a configured runtime driver and monitor reset";
      homingCycle = new HomingCycle(driver, homingAxes, homingSides);
      homingCycle.start();
      return null;
    }
    return moveAxes([for (axisValue in axes) new AxisTarget(axisValue.id, axisValue.homePosition)], options);
  }

  /**
   * Plans a bounded timed jog in one logical axis. The endpoint is clamped to
   * the authored axis limits, while all physical joints in a coordinated axis
   * group receive the same logical displacement and scaled velocity.
   */
  public function jog(axisId:String, velocity:Float, durationSeconds:Float,
      ?maxAcceleration:Float):Null<Trajectory> {
    checkSnapshot();
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
      var plannedJog = axisPlanner.planJog(start, axisValue, velocity,
        durationSeconds, acceleration).trajectory;
      jogTrajectories.set(plannedJog, true);
      return plannedJog;
    }
    return replaceMotion(planJog(), MotionRequestCapture.jog(axisValue.id,
      velocity, durationSeconds, acceleration));
  }

  /**
   * Changes a running jog on the same axis at the committed plan state.
   */
  function continueJog(axisValue:MotionAxis, velocity:Float, durationSeconds:Float,
      acceleration:Float):Null<Trajectory> {
    var executing = activeTrajectory;
    if (executing == null || session.jogAxis != axisValue.id || session.isHolding() || session.isStopping() ||
        session.hasPending() || queuedTrajectories.length > 0)
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
    if (executing == null || session.isHolding() || session.isStopping() ||
        session.hasPending() || queuedTrajectories.length > 0)
      return null;
    var smooth = executing;
    if (smooth == null || smooth.segments()[0].coefficients[0].length < 3) return null;
    return trySmoothReplacement(executing, null, state ->
      axisPlanner.planRetarget(state, targets, options).trajectory);
  }

  function trySmoothReplacement(executing:Trajectory, jogAxis:Null<String>,
      planFromState:TrajectoryState -> Trajectory):Null<Trajectory> {
    if (executing == null) return null;
    try {
      return stream.replaceWithRetry(executing, syncFromRuntime,
        replacementOwnerPeriodSeconds, replacementMarginOwnerPeriods, planFromState,
        (planned, state, observation, anchorNs) ->
          submitSmoothReplacement(planned, state, observation, anchorNs, jogAxis));
    } catch (error:RobotRuntimeError) {
      session.reject();
      throw error;
    }
  }

  function submitSmoothReplacement(planned:Trajectory, state:TrajectoryState,
      observation:RobotSnapshot, anchorNs:Int64, jogAxis:Null<String>):Trajectory {
    var tag = stream.submitSmoothReplacement(planned, state, observation,
      anchorNs, modelRevision, calibrationRevision, jointTolerances,
      jogAxis == null ? robotkit.execution.ExecutionPlanPurpose.Program : robotkit.execution.ExecutionPlanPurpose.Jog);
    if (jogAxis != null) jogTrajectories.set(planned, true);
    setActive(planned);
    session.jogAxis = jogAxis;
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
    updateCount++;
    var homing = homingCycle;
    if (homing != null && homing.isActive()) return homing.update(dt);
    checkSnapshot();
    if (session.isHolding() || session.isStopping()) {
      // No refills while stopping: beginStop() already queued enough path for
      // the whole stop, and a late chunk could otherwise land after the stop
      // finished and be taken as a resume.
      var stopping = activeTrajectory;
      if (stopping != null && trajectoryFinishedInRuntime(syncFromRuntime()))
        completeActiveTrajectory();
      advanceStop();
      if (session.isHolding() || session.isStopping()) return false;
    }
    if (activeTrajectory == null) activateNextTrajectory();
    if (activeTrajectory == null) return false;
    var trajectory = activeTrajectory;
    if (trajectory.durationSeconds() <= 0.0 || activeStationary) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    var observation = syncFromRuntime();
    if (session.isStopping()) return true;
    if (trajectoryFinishedInRuntime(observation)) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    if (nativeRefillDeferred) nativeRefillDeferred = false;
    // One submission an update: motion that started during this update has submitted.
    else if (filledAtUpdate != updateCount &&
        (!stream.submitted || stream.shouldRefill(dt, fixedTimestepSeconds)))
      fillNativeWindow();
    return true;
  }

  public function stop(?mode:StopMode = StopMode.Normal):Void {
    abort(mode);
  }

  /** Starts executing `trajectoryValue` as a fresh, un-retimed plan. */
  function setActive(trajectoryValue:Trajectory):Void {
    // Pre-timed/sample trajectories still belong to this machine's motion session.
    if (trajectoryValue.controlAcceleration == null) {
      var caps = [for (_ in trajectoryValue.evaluate(0.0).positions) 0.0];
      for (axis in axes) for (index in 0...axis.jointIndices.length)
        caps[axis.jointIndices[index]] = axis.maxAcceleration * Math.abs(axis.jointScale(index));
      trajectoryValue.controlAcceleration = caps;
    }
    activeTrajectory = trajectoryValue;
    var segments = trajectoryValue.segments();
    activeStationary = stationarySegments(segments);
    session.begin();
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
    if (homingCycle != null && homingCycle.isActive()) throw "Ordinary motion cannot queue during homing";
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
    if (session.isHolding() || session.isStopping() || activeTrajectory != null ||
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
    session.clear();
    bufferedTotalSeconds = 0.0;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = null;
    nativeRefillDeferred = false;
    stream.clear();
  }

  /** Keep two seconds of motion queued so HOLD can slow along the path. */
  function fillNativeWindow():Void {
    filledAtUpdate = updateCount;
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    var jerkUnchecked = pathJerkUnchecked.exists(trajectoryValue) &&
      pathJerkUnchecked.get(trajectoryValue) == true;
    try {
      stream.fill(session,
        (first, last, tag, startNs, _) -> stream.motionSubmission(
          trajectoryValue, first, last, tag, startNs, modelRevision,
          calibrationRevision, jerkUnchecked, jointTolerances,
          jogTrajectories.exists(trajectoryValue) ? robotkit.execution.ExecutionPlanPurpose.Jog : robotkit.execution.ExecutionPlanPurpose.Program),
        (first, last, error) ->
          'plan chunk [$first,$last] of ${trajectoryValue.segments().length}: $error');
    } catch (error:Dynamic) {
      session.reject();
      throw error;
    }
  }

  function get_elapsedSeconds():Float return stream.elapsedSeconds;

  function syncFromRuntime():RobotSnapshot
    return observe(activeTrajectory == null ? robot.snapshot() : stream.sync());

  function trajectoryFinishedInRuntime(observation:RobotSnapshot):Bool
    return activeTrajectory != null && stream.finishedMotion(observation);

  function completeActiveTrajectory():Void {
    if (activeTrajectory == null) return;
    bufferedCompletedSeconds += activeTrajectory.durationSeconds();
    pathJerkUnchecked.remove(activeTrajectory);
    jogTrajectories.remove(activeTrajectory);
    activeTrajectory = null;
    activeStationary = false;
    stream.clear();
    session.completed();
    activateNextTrajectory();
  }

  function observe(snapshot:RobotSnapshot):RobotSnapshot
    return stream.observe(session, snapshot);

  function checkSnapshot():Void observe(robot.snapshot());

  function submitCommand(command:RobotCommand):Void stream.submit(session, command);

  function stopRobot(mode:StopMode):Void stream.stop(session, mode);

  function clearBufferedMotionAfterFault():Void {
    activeTrajectory = null;
    activeStationary = false;
    queuedTrajectories = [];
    bufferedTotalSeconds = 0.0;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = null;
    nativeRefillDeferred = false;
    stream.clear();
  }

  function discardNativeTrajectory(value:Trajectory):Void {
    pathJerkUnchecked.remove(value);
    jogTrajectories.remove(value);
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

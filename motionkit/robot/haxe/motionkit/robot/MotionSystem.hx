package motionkit.robot;

import haxe.Int64;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.axis.MotionAxis;
import motionkit.path.PathPoint;
import motionkit.path.GeometricPath;
import motionkit.planner.JogProfile;
import motionkit.planner.LineLookaheadPlanner;
import motionkit.planner.PathPlanningOptions;
import motionkit.planner.TrajectoryPlanner;
import motionkit.planner.TrapezoidalPlanner;
import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;
import motionkit.trajectory.TimeScaledTrajectory;
import motionkit.trajectory.TimeScaling;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.TrajectoryState;
import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.StopMode;
import robotkit.world.TrajectoryChunk;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.TrajectorySegment;
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
  public final axes:Array<MotionAxis>;
  public final fixedTimestepSeconds:Float;
  public final replacementMarginOwnerPeriods:Int;
  public final replacementOwnerPeriodSeconds:Float;
  public final planner:TrajectoryPlanner;
  public final linePlanner:LineLookaheadPlanner;
  /** Trajectory being executed; a re-timed copy of activeSource after a hold or resume. */
  var activeTrajectory:Null<JointTrajectory> = null;
  /** The planned trajectory behind activeTrajectory. */
  var activeSource:Null<JointTrajectory> = null;
  /** Maps activeTrajectory time back to activeSource time when it was re-timed. */
  var activeTiming:Null<TimeScaledTrajectory> = null;
  var queuedTrajectories:Array<JointTrajectory> = [];
  /** Native polynomials backing smooth axis moves until the public API migrates. */
  var nativeTrajectories:Array<{trajectory:JointTrajectory, native:Trajectory}> = [];
  var elapsedSeconds:Float = 0.0;
  var held:Bool = false;
  /** A native lifecycle command must reach the owner before another plan. */
  var nativeRefillDeferred:Bool = false;
  var bufferedTotalSeconds:Float = 0.0;
  var bufferedCompletedSeconds:Float = 0.0;
  var plannedEndPositions:Null<Array<Float>> = null;
  var trajectorySubmitted:Bool = false;
  /** Source-sample index used as the boundary of the next native chunk. */
  var trajectoryNextSampleIndex:Int = 0;
  var trajectoryChunkEndSeconds:Float = 0.0;
  var trajectoryChunkStartSeconds:Float = 0.0;
  var nextTrajectoryTag:Int64 = Int64.ofInt(1);
  var trajectoryChunkReferences:Map<String, TrajectoryChunkReference> = new Map();
  var trajectoryFinalTag:Int64 = Int64.ofInt(0);
  var trajectoryFinalEndSeconds:Float = 0.0;
  var resumeRequested:Bool = false;
  /** True while the position-target path plays a host-side stop. */
  var hostStopping:Bool = false;
  /** True while stopping so that deferred work can start from rest. */
  var stoppingForReplacement:Bool = false;
  /** Work deferred until a replacement stop reaches rest, in order. */
  var afterStop:Array<Void -> Void> = [];
  /** Logical axis of the active jog, which a further jog can change without stopping. */
  var activeJogAxis:Null<String> = null;
  /** Per-joint acceleration limits from the logical axes; zero is unconstrained. */
  final jointAccelerationLimits:Array<Float>;
  final modelRevision:Int64;
  final calibrationRevision:Int64;

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
    this.planner = planner == null ? new TrapezoidalPlanner(fixedTimestepSeconds) : planner;
    this.linePlanner = new LineLookaheadPlanner(fixedTimestepSeconds);
    this.axes = [];
    var axisIds = new Map<String, Bool>();
    for (axisBlueprint in blueprint.axes) {
      var axis = new MotionAxis(axisBlueprint, description.joints);
      if (axisIds.exists(axis.id)) throw 'Duplicate motion axis "${axis.id}"';
      axisIds.set(axis.id, true);
      this.axes.push(axis);
    }
    this.jointAccelerationLimits = [for (_ in description.joints) 0.0];
    for (axisValue in axes) {
      if (axisValue.maxAcceleration <= 0.0) continue;
      var scales = [for (_ in description.joints) 0.0];
      axisValue.writeLogicalDelta(scales, 1.0);
      for (joint in axisValue.jointIndices) {
        var limit = axisValue.maxAcceleration * Math.abs(scales[joint]);
        var current = jointAccelerationLimits[joint];
        jointAccelerationLimits[joint] = current <= 0.0 ? limit : Math.min(current, limit);
      }
    }
  }

  public function axis(id:String):Null<MotionAxis> {
    for (value in axes) if (value.id == id) return value;
    return null;
  }

  /** True while a trajectory is active, held, or waiting to start after a stop. */
  public function isMoving():Bool return activeTrajectory != null || afterStop.length > 0;

  public function trajectory():Null<JointTrajectory> return activeTrajectory;

  /** Number of active and waiting trajectories currently owned by the system. */
  public function queueDepth():Int
    return (activeTrajectory == null ? 0 : 1) + queuedTrajectories.length;

  /** Remaining duration across the active trajectory and all queued trajectories. */
  public function queuedDurationSeconds():Float {
    var result = activeTrajectory == null ? 0.0 :
      Math.max(0.0, activeTrajectory.durationSeconds - elapsedSeconds);
    for (trajectoryValue in queuedTrajectories) result += trajectoryValue.durationSeconds;
    return result;
  }

  /** Completion fraction of the currently buffered motion, in the range [0, 1]. */
  public function progress():Float {
    if (activeTrajectory != null && usesTrajectoryChunks(activeTrajectory))
      syncFromRuntime();
    if (bufferedTotalSeconds <= 0.0) return 1.0;
    var source = activeSource;
    var completed = bufferedCompletedSeconds + (source == null ? 0.0
      : Math.min(source.durationSeconds, activeSourceSeconds()));
    return Math.min(1.0, Math.max(0.0, completed / bufferedTotalSeconds));
  }

  public function isHolding():Bool return held;

  /**
   * Plans a coordinated move while leaving unspecified axes at their current
   * positions, replacing any buffered motion. While the machine is moving it
   * first stops along its current path; the move is then planned from where
   * it came to rest and null is returned, as the plan does not exist yet.
   */
  public function moveAxes(targets:Array<AxisTarget>, ?options:MotionOptions):Null<JointTrajectory> {
    var retargeted = retargetNativeAxes(targets, options);
    if (retargeted != null) return retargeted;
    var planned = planAxesFrom(robot.snapshot().positions.toArray(), targets, options);
    return replaceMotion(planned,
      () -> planAxesFrom(robot.snapshot().positions.toArray(), targets, options));
  }

  /** Adds a coordinated axis move behind all motion already in the buffer. */
  public function queueAxes(targets:Array<AxisTarget>, ?options:MotionOptions):Null<JointTrajectory> {
    if (afterStop.length > 0) {
      afterStop.push(() -> queueAxes(targets, options));
      return null;
    }
    var trajectoryValue = planAxesFrom(planningStartPositions(), targets, options);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds an already planned joint trajectory to the execution buffer. */
  public function queueTrajectory(trajectoryValue:JointTrajectory):Void {
    if (trajectoryValue == null) throw "Queued trajectory is required";
    var jointCount = robot.description().joints.length;
    if (trajectoryValue.jointCount != jointCount)
      throw 'Queued trajectory has ${trajectoryValue.jointCount} joints; robot has $jointCount';
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
      ?options:MotionOptions):Null<JointTrajectory> {
    var planned = planLinearPathFrom(robot.snapshot().positions.toArray(), target, feed, options);
    return replaceMotion(planned,
      () -> planLinearPathFrom(robot.snapshot().positions.toArray(), target, feed, options));
  }

  /** Adds a straight Cartesian move behind all motion already in the buffer. */
  public function queueLinear(target:PathPoint, feed:Feed,
      ?options:MotionOptions):Null<JointTrajectory> {
    if (afterStop.length > 0) {
      afterStop.push(() -> queueLinear(target, feed, options));
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
      ?motionOptions:MotionOptions):JointTrajectory {
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
      ?motionOptions:MotionOptions):Null<JointTrajectory> {
    if (afterStop.length > 0) {
      afterStop.push(() -> queuePath(path, pathOptions, motionOptions));
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
    resumeRequested = false;
    if (activeTrajectory != null && usesTrajectoryChunks(activeTrajectory)) {
      robot.submit(RobotCommand.Hold);
      return;
    }
    beginStop();
  }

  /**
   * Resumes a held buffer from where it stopped, speeding back up along the
   * path. If the hold is still slowing down, it resumes once at rest.
   */
  public function resume():Void {
    if (!held) return;
    if (activeTrajectory != null && usesTrajectoryChunks(activeTrajectory)) {
      if (trajectoryFinishedInRuntime(syncFromRuntime())) {
        completeActiveTrajectory();
        resumeFromRest();
        return;
      }
      robot.submit(RobotCommand.Resume);
      nativeRefillDeferred = true;
      held = false;
      resumeRequested = false;
      return;
    }
    resumeRequested = true;
    advanceStop();
  }

  /**
   * Aborts buffered motion. A normal abort slows to rest along the current
   * path before discarding it; an emergency stop acts immediately.
   */
  public function abort(?mode:StopMode = StopMode.Normal):Void {
    if (mode == StopMode.Normal && isMotionInProgress()) {
      if (activeTrajectory != null && usesTrajectoryChunks(activeTrajectory)) {
        robot.submit(RobotCommand.Abort);
        clearBufferedMotion();
        return;
      }
      queuedTrajectories = [];
      afterStop = [() -> clearBufferedMotion()];
      held = false;
      resumeRequested = false;
      stoppingForReplacement = true;
      beginStop();
      return;
    }
    clearBufferedMotion();
    robot.stop(mode);
  }

  /**
   * Starts `planned` now when the machine is at rest. Otherwise stops along
   * the current path and starts a fresh `replan` from where it came to rest.
   */
  function replaceMotion(planned:JointTrajectory, replan:Void -> JointTrajectory,
      ?jogAxis:String):Null<JointTrajectory> {
    if (!isMotionInProgress()) {
      clearBufferedMotion();
      beginImmediate(planned);
      activeJogAxis = jogAxis;
      return planned;
    }
    discardNativeTrajectory(planned);
    for (queued in queuedTrajectories) discardNativeTrajectory(queued);
    queuedTrajectories = [];
    afterStop = [() -> {
      clearBufferedMotion();
      beginImmediate(replan());
      activeJogAxis = jogAxis;
    }];
    held = false;
    resumeRequested = false;
    stoppingForReplacement = true;
    beginStop();
    return null;
  }

  /**
   * Whether the machine is in motion or may still be: a trajectory that has
   * started and not finished, or a stop still slowing down.
   */
  function isMotionInProgress():Bool {
    if (hostStopping || stoppingForReplacement) return true;
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return false;
    if (usesTrajectoryChunks(trajectoryValue) && syncFromRuntime().trajectoryActive) return true;
    return elapsedSeconds > 1e-9 && elapsedSeconds < trajectoryValue.durationSeconds - 1e-9 &&
      !(held && stopSettled());
  }

  /** Slows the active trajectory to rest along its path. */
  function beginStop():Void {
    var trajectoryValue = activeTrajectory, source = activeSource;
    if (trajectoryValue == null || source == null) return;
    if (usesTrajectoryChunks(trajectoryValue)) {
      syncFromRuntime();
      robot.stop(StopMode.Normal);
      return;
    }
    if (hostStopping) return;
    // Without a runtime queue, play the same path-following stop host-side.
    var sourceTime = activeSourceSeconds();
    if (elapsedSeconds <= 1e-9 || sourceTime >= source.durationSeconds - 1e-9) return;
    retimeActive(TimeScaling.stop(source, sourceTime, activeRate(),
      jointAccelerationLimits, fixedTimestepSeconds));
    hostStopping = true;
  }

  /** True once a requested stop has reached rest. */
  function stopSettled():Bool {
    if (hostStopping) return false;
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !usesTrajectoryChunks(trajectoryValue)) return true;
    return !syncFromRuntime().trajectoryActive;
  }

  /** Runs deferred work or a pending resume once a stop has reached rest. */
  function advanceStop():Void {
    if (!stopSettled()) return;
    if (stoppingForReplacement && (!held || resumeRequested)) {
      stoppingForReplacement = false;
      held = false;
      resumeRequested = false;
      var actions = afterStop;
      afterStop = [];
      for (action in actions) action();
      return;
    }
    if (held && resumeRequested) resumeFromRest();
  }

  /** Speeds the active trajectory back up from rest along its path. */
  function resumeFromRest():Void {
    held = false;
    resumeRequested = false;
    var source = activeSource;
    if (activeTrajectory == null || source == null) {
      activateNextTrajectory();
      return;
    }
    var sourceTime = activeSourceSeconds();
    if (sourceTime >= source.durationSeconds - 1e-9) {
      completeActiveTrajectory();
      return;
    }
    var timing = TimeScaling.start(source, sourceTime, jointAccelerationLimits,
      fixedTimestepSeconds);
    retimeActive(timing);
    if (usesTrajectoryChunks(timing.trajectory)) submitActiveTrajectoryChunk();
  }

  function planLinearPathFrom(start:Array<Float>, target:PathPoint, feed:Feed,
      options:Null<MotionOptions>):JointTrajectory {
    if (target == null || feed == null) throw "Linear move needs a target and feed";
    var xAxis = requireAxis("x");
    var yAxis = requireAxis("y");
    var zAxis = requireAxis("z");
    var current = new PathPoint(xAxis.logicalPosition(start), yAxis.logicalPosition(start),
      zAxis.logicalPosition(start));
    var chosen = options == null ? new MotionOptions(feed.value, 0.0, 0.0) : options;
    var maxVelocity = chosen.maxVelocity <= 0.0
      ? feed.value : Math.min(feed.value, chosen.maxVelocity);
    var resolved = new MotionOptions(maxVelocity, chosen.maxAcceleration, chosen.maxJerk);
    return planPathFrom(start, GeometricPath.lines([current, target]),
      PathPlanningOptions.exactStopMode(), resolved);
  }

  function planPathFrom(start:Array<Float>, path:GeometricPath,
      pathOptions:Null<PathPlanningOptions>, motionOptions:Null<MotionOptions>):JointTrajectory {
    if (path == null) throw "Cartesian path is required";
    var xAxis = requireAxis("x");
    var yAxis = requireAxis("y");
    var zAxis = requireAxis("z");
    var first = path.primitives[0].pointAt(0.0);
    var starts = [xAxis.logicalPosition(start), yAxis.logicalPosition(start),
      zAxis.logicalPosition(start)];
    var startCoordinates = [first.x, first.y, first.z];
    for (i in 0...3) {
      if (Math.abs(starts[i] - startCoordinates[i]) > 1e-8)
        throw 'Cartesian path starts at ${startCoordinates[i]} but axis ${["x", "y", "z"][i]} is at ${starts[i]}';
    }

    var limits = resolvePathLimits(path, [xAxis, yAxis, zAxis],
      motionOptions == null ? new MotionOptions() : motionOptions);
    validatePathLimits(path, [xAxis, yAxis, zAxis]);
    var cartesian = linePlanner.planPath(path, limits, pathOptions);
    var nativeCartesian = robot.capabilities().supportsExecutionPlans
      ? linePlanner.planNativePath(path, limits, pathOptions) : null;
    var mapped:Array<JointTrajectorySample> = [];
    for (sample in cartesian.samples) {
      var positions = start.copy();
      var velocities = [for (_ in start) 0.0];
      var accelerations = [for (_ in start) 0.0];
      var cartesianPosition = nativeCartesian == null ? sample.positions
        : nativeCartesian.evaluate(sample.timeSeconds).positions;
      xAxis.writeLogicalPosition(positions, cartesianPosition[0]);
      yAxis.writeLogicalPosition(positions, cartesianPosition[1]);
      zAxis.writeLogicalPosition(positions, cartesianPosition[2]);
      writeLogicalVector(xAxis, velocities, sample.velocities[0]);
      writeLogicalVector(yAxis, velocities, sample.velocities[1]);
      writeLogicalVector(zAxis, velocities, sample.velocities[2]);
      writeLogicalVector(xAxis, accelerations, sample.accelerations[0]);
      writeLogicalVector(yAxis, accelerations, sample.accelerations[1]);
      writeLogicalVector(zAxis, accelerations, sample.accelerations[2]);
      mapped.push(new JointTrajectorySample(sample.timeSeconds, positions, velocities,
        accelerations));
    }
    if (nativeCartesian != null) nativeCartesian.dispose();
    var result = new JointTrajectory(mapped);
    if (robot.capabilities().supportsExecutionPlans && mapped.length > 1) {
      var native = Trajectory.fromPositionSamples(
        [for (sample in mapped) sample.timeSeconds],
        [for (sample in mapped) sample.positions]);
      nativeTrajectories.push({trajectory: result, native: native});
    }
    return result;
  }

  function resolvePathLimits(path:GeometricPath, directAxes:Array<MotionAxis>,
      options:MotionOptions):MotionLimits {
    var maxVelocity = options.maxVelocity;
    var maxAcceleration = options.maxAcceleration;
    for (primitive in path.primitives) {
      var length = primitive.length();
      if (length <= 1e-12) continue;
      for (sampleIndex in 0...65) {
        var tangent = primitive.tangentAt(length * sampleIndex / 64.0);
        var curvature = Math.abs(primitive.curvatureAt(length * sampleIndex / 64.0));
        var normal = [
          curvature <= 1e-12 ? 0.0 : -tangent[1],
          curvature <= 1e-12 ? 0.0 : tangent[0],
          0.0
        ];
        for (i in 0...3) {
          var axisAcceleration = directAxes[i].maxAcceleration;
          if (axisAcceleration > 0.0 && curvature > 1e-12) {
            var normalCoefficient = Math.abs(normal[i]) * curvature;
            if (normalCoefficient > 1e-12) {
              var centripetalVelocity = Math.sqrt(axisAcceleration / normalCoefficient);
              maxVelocity = maxVelocity <= 0.0 ? centripetalVelocity :
                Math.min(maxVelocity, centripetalVelocity);
            }
          }
        }
        for (i in 0...3) {
          var fraction = Math.min(1.0, Math.abs(tangent[i]) + 1e-6);
          var axisVelocity = directAxes[i].maxVelocity;
          var axisAcceleration = directAxes[i].maxAcceleration;
          if (fraction > 1e-12 && axisVelocity > 0.0) {
            var projectedVelocity = axisVelocity / fraction;
            maxVelocity = maxVelocity <= 0.0 ? projectedVelocity : Math.min(maxVelocity,
              projectedVelocity);
          }
          if (axisAcceleration > 0.0) {
            var centripetal = Math.abs(normal[i] * curvature) * maxVelocity * maxVelocity;
            var available = axisAcceleration - centripetal;
            if (fraction > 1e-12 && available > 0.0) {
              var projectedAcceleration = available / fraction;
              maxAcceleration = maxAcceleration <= 0.0 ? projectedAcceleration :
                Math.min(maxAcceleration, projectedAcceleration);
            }
          }
        }
      }
    }
    return new MotionLimits(maxVelocity, maxAcceleration, options.maxJerk);
  }

  function validatePathLimits(path:GeometricPath, directAxes:Array<MotionAxis>):Void {
    for (primitive in path.primitives) {
      var length = primitive.length();
      for (sampleIndex in 0...65) {
        var point = primitive.pointAt(length * sampleIndex / 64.0);
        var coordinates = [point.x, point.y, point.z];
        for (i in 0...3) {
          if (coordinates[i] < directAxes[i].lowerLimit ||
              coordinates[i] > directAxes[i].upperLimit)
            throw 'Axis "${["x", "y", "z"][i]}" path point ${coordinates[i]} is outside its limits';
        }
      }
    }
  }

  static function writeLogicalVector(axis:MotionAxis, joints:Array<Float>, value:Float):Void {
    axis.writeLogicalDelta(joints, value);
  }

  /** Software homing for the bootstrap: move to each authored home coordinate. */
  public function home(?options:MotionOptions):Null<JointTrajectory> {
    return moveAxes([for (axisValue in axes) new AxisTarget(axisValue.id, axisValue.homePosition)], options);
  }

  /**
   * Plans a bounded timed jog in one logical axis. The endpoint is clamped to
   * the authored axis limits, while all physical joints in a coordinated axis
   * group receive the same logical displacement and scaled velocity.
   */
  public function jog(axisId:String, velocity:Float, durationSeconds:Float,
      ?maxAcceleration:Float):Null<JointTrajectory> {
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
    function planJog():JointTrajectory {
      var start = robot.snapshot().positions.toArray();
      var startLogical = axisValue.logicalPosition(start);
      var requestedEnd = startLogical + velocity * durationSeconds;
      var endLogical = Math.max(axisValue.lowerLimit,
        Math.min(axisValue.upperLimit, requestedEnd));
      return planLogical(start, [axisValue], [endLogical],
        new MotionLimits(Math.abs(velocity), acceleration));
    }
    return replaceMotion(planJog(), planJog, axisValue.id);
  }

  /**
   * Changes a running jog on the same axis without stopping. Native queues
   * replace at the committed state; position-streaming backends continue from
   * the current host sample. Returns null if continuation is unavailable.
   */
  function continueJog(axisValue:MotionAxis, velocity:Float, durationSeconds:Float,
      acceleration:Float):Null<JointTrajectory> {
    var executing = activeTrajectory;
    if (executing == null || activeJogAxis != axisValue.id || held || stoppingForReplacement ||
        hostStopping || afterStop.length > 0 || queuedTrajectories.length > 0)
      return null;
    var smooth = nativeFor(executing);
    if (smooth != null && usesTrajectoryChunks(executing)) {
      return trySmoothReplacement(executing, axisValue.id, state -> {
        var target = state.positions.copy();
        var logical = axisValue.logicalPosition(target);
        var end = Math.max(axisValue.lowerLimit,
          Math.min(axisValue.upperLimit, logical + velocity * durationSeconds));
        axisValue.writeLogicalPosition(target, end);
        var currentLogicalVelocity = axisValue.logicalPosition(
          [for (joint in 0...state.velocities.length)
            state.positions[joint] + state.velocities[joint]]) - logical;
        return nativeLogicalPlan(state.positions, state.velocities,
          state.accelerations, target,
          new MotionLimits(Math.max(Math.abs(velocity), Math.abs(currentLogicalVelocity)),
            acceleration));
      });
    }
    if (usesTrajectoryChunks(executing)) return null;
    var sampleSeconds = elapsedSeconds;
    if (sampleSeconds >= executing.durationSeconds - fixedTimestepSeconds) return null;

    var start = executing.sample(sampleSeconds).positions;
    var startLogical = axisValue.logicalPosition(start);
    var startVelocity = logicalVelocityAt(executing, axisValue, sampleSeconds);
    var requestedEnd = Math.max(axisValue.lowerLimit,
      Math.min(axisValue.upperLimit, startLogical + velocity * durationSeconds));
    var profile = new JogProfile(startLogical, startVelocity, requestedEnd, Math.abs(velocity),
      acceleration);
    // Braking from the current speed may not fit before a travel limit.
    if (profile.endPosition < axisValue.lowerLimit - 1e-12 ||
        profile.endPosition > axisValue.upperLimit + 1e-12)
      return null;
    var samples:Array<JointTrajectorySample> = [];
    var count = Std.int(Math.max(1.0, Math.ceil(profile.durationSeconds / fixedTimestepSeconds)));
    for (index in 0...(count + 1)) {
      var time = index == count ? profile.durationSeconds : index * fixedTimestepSeconds;
      var positions = start.copy();
      var logical = Math.max(axisValue.lowerLimit,
        Math.min(axisValue.upperLimit, profile.positionAt(time)));
      axisValue.writeLogicalPosition(positions, logical);
      var velocities = [for (_ in start) 0.0];
      axisValue.writeLogicalDelta(velocities, profile.velocityAt(time));
      samples.push(new JointTrajectorySample(time, positions, velocities));
    }
    var trajectoryValue = new JointTrajectory(samples);

    setActive(trajectoryValue);
    bufferedTotalSeconds = trajectoryValue.durationSeconds;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(trajectoryValue);
    activeJogAxis = axisValue.id;
    return trajectoryValue;
  }

  function retargetNativeAxes(targets:Array<AxisTarget>,
      options:Null<MotionOptions>):Null<JointTrajectory> {
    var executing = activeTrajectory;
    if (executing == null || held || stoppingForReplacement || hostStopping ||
        afterStop.length > 0 || queuedTrajectories.length > 0 ||
        !usesTrajectoryChunks(executing))
      return null;
    var smooth = nativeFor(executing);
    if (smooth == null || smooth.segments()[0].coefficients[0].length < 3) return null;
    return trySmoothReplacement(executing, null, state -> {
      var target = state.positions.copy();
      var seen = new Map<String, Bool>();
      for (requested in targets) {
        if (requested == null) throw "Axis move cannot contain a null target";
        var axisValue = axis(requested.axis);
        if (axisValue == null) throw 'Unknown motion axis "${requested.axis}"';
        if (seen.exists(requested.axis)) throw 'Axis move targets "${requested.axis}" more than once';
        if (requested.position < axisValue.lowerLimit ||
            requested.position > axisValue.upperLimit)
          throw 'Axis "${requested.axis}" target ${requested.position} is outside its limits';
        axisValue.writeLogicalPosition(target, requested.position);
        seen.set(requested.axis, true);
      }
      var limits = resolveLimits(targets,
        options == null ? new MotionOptions() : options);
      return nativeLogicalPlan(state.positions, state.velocities,
        state.accelerations, target, limits);
    });
  }

  function trySmoothReplacement(executing:JointTrajectory, jogAxis:Null<String>,
      planFromState:TrajectoryState -> JointTrajectory):Null<JointTrajectory> {
    var smooth = nativeFor(executing);
    if (smooth == null) return null;
    for (_ in 0...2) {
      var observation = syncFromRuntime();
      if (!observation.trajectoryActive ||
          Int64.compare(observation.activePlanId, Int64.ofInt(0)) == 0)
        return null;
      var startNs = Int64.sub(observation.trajectoryTimeNs,
        observation.trajectoryTagTimeNs);
      var marginNs = Trajectory.nanoseconds(
        replacementOwnerPeriodSeconds * replacementMarginOwnerPeriods);
      var anchorNs = Int64.add(observation.committedUntilNs, marginNs);
      var localNs = Int64.sub(anchorNs, startNs);
      var localSeconds = Int64.toFloat(localNs) * 1e-9;
      if (localSeconds < 0.0 ||
          localSeconds >= smooth.durationSeconds() - replacementOwnerPeriodSeconds)
        return null;
      var state = smooth.evaluate(localSeconds);
      var planned = planFromState(state);
      try {
        return submitSmoothReplacement(planned, state, observation, anchorNs, jogAxis);
      } catch (error:Dynamic) {
        discardNativeTrajectory(planned);
        var runtimeError:Null<RobotRuntimeError> = Std.isOfType(error, RobotRuntimeError)
          ? cast error : null;
        if (runtimeError == null || runtimeError.status !=
            RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE)
          throw error;
      }
    }
    return null;
  }

  function submitSmoothReplacement(planned:JointTrajectory, state:TrajectoryState,
      observation:RobotSnapshot, anchorNs:Int64, jogAxis:Null<String>):JointTrajectory {
    var replacement = nativeFor(planned);
    var tag = nextTrajectoryTag;
    var segments = [for (segment in replacement.segments())
      new TrajectorySegment(segment.timeFromStartNs, segment.durationNs,
        segment.coefficients)];
    if (segments.length > 128) throw "Smooth replacement exceeds one runtime submission";
    robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
      modelRevision, calibrationRevision, 1, state.positions, state.velocities,
      state.accelerations, segments, observation.activePlanId,
      anchorNs)));
    nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
    setActive(planned);
    activeJogAxis = jogAxis;
    bufferedTotalSeconds = planned.durationSeconds;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(planned);
    trajectorySubmitted = true;
    trajectoryNextSampleIndex = planned.samples.length - 1;
    trajectoryChunkStartSeconds = 0.0;
    trajectoryChunkEndSeconds = planned.durationSeconds;
    trajectoryFinalTag = tag;
    trajectoryFinalEndSeconds = planned.durationSeconds;
    trajectoryChunkReferences.set(Int64.toStr(tag),
      new TrajectoryChunkReference(planned, 0.0));
    return planned;
  }

  /**
   * Logical velocity of `trajectoryValue` at `timeSeconds`. A segment's chord
   * velocity is exact at its midpoint, so this interpolates between the
   * chords of the neighbouring segments rather than taking the chord ahead,
   * which would be half a sample late while the trajectory accelerates.
   */
  static function logicalVelocityAt(trajectoryValue:JointTrajectory, axisValue:MotionAxis,
      timeSeconds:Float):Float {
    var samples = trajectoryValue.samples;
    var midpoints:Array<Float> = [];
    var chords:Array<Float> = [];
    for (index in 0...(samples.length - 1)) {
      var before = samples[index];
      var after = samples[index + 1];
      if (after.timeSeconds <= before.timeSeconds) continue;
      midpoints.push(0.5 * (before.timeSeconds + after.timeSeconds));
      chords.push((axisValue.logicalPosition(after.positions) -
        axisValue.logicalPosition(before.positions)) / (after.timeSeconds - before.timeSeconds));
    }
    if (chords.length == 0) return 0.0;
    if (chords.length == 1 || timeSeconds <= midpoints[0]) return chords[0];
    var last = chords.length - 1;
    if (timeSeconds >= midpoints[last]) return chords[last];
    var index = 0;
    while (index + 1 < last && midpoints[index + 1] <= timeSeconds) index++;
    var alpha = (timeSeconds - midpoints[index]) / (midpoints[index + 1] - midpoints[index]);
    return chords[index] + (chords[index + 1] - chords[index]) * alpha;
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
      if (stopping != null) {
        if (usesTrajectoryChunks(stopping)) {
          if (trajectoryFinishedInRuntime(syncFromRuntime()))
            completeActiveTrajectory();
        } else if (hostStopping) {
          // The final stop sample is applied by the step after it is sent, so
          // the stop only counts as settled one update later; planning from
          // rest before then would start from a stale pose.
          if (elapsedSeconds > stopping.durationSeconds) {
            hostStopping = false;
          } else {
            submitPositionSample(stopping.sample(elapsedSeconds));
            elapsedSeconds = elapsedSeconds >= stopping.durationSeconds
              ? stopping.durationSeconds + dt
              : Math.min(stopping.durationSeconds, elapsedSeconds + dt);
          }
        }
      }
      advanceStop();
      if (held || stoppingForReplacement) return hostStopping;
    }
    if (activeTrajectory == null) activateNextTrajectory();
    if (activeTrajectory == null) return false;
    var trajectory = activeTrajectory;
    if (trajectory.durationSeconds <= 0.0) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    if (usesTrajectoryChunks(trajectory)) {
      var observation = syncFromRuntime();
      if (stoppingForReplacement) return true;
      if (trajectoryFinishedInRuntime(observation)) {
        completeActiveTrajectory();
        return activeTrajectory != null;
      }
      if (nativeRefillDeferred) nativeRefillDeferred = false;
      else if (!trajectorySubmitted || shouldRefillTrajectory(dt)) fillNativeWindow();
    } else {
      submitPositionSample(trajectory.sample(elapsedSeconds));
    }
    if (!usesTrajectoryChunks(trajectory) && elapsedSeconds >= trajectory.durationSeconds) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    if (!usesTrajectoryChunks(trajectory))
      elapsedSeconds = Math.min(trajectory.durationSeconds, elapsedSeconds + dt);
    return true;
  }

  public function stop(?mode:StopMode = StopMode.Normal):Void {
    abort(mode);
  }

  function submitPositionSample(sample:JointTrajectorySample):Void {
    var targets:Array<JointTarget> = [];
    for (i in 0...sample.positions.length)
      targets.push(JointTarget.position(i, sample.positions[i]));
    robot.submit(RobotCommand.JointTargets(targets, null));
  }

  /** Time along the planned trajectory reached by the executed one. */
  function activeSourceSeconds():Float {
    var timing = activeTiming;
    return timing == null ? elapsedSeconds : timing.sourceTimeAt(elapsedSeconds);
  }

  /** Clock rate of the executed trajectory relative to its plan. */
  function activeRate():Float {
    var timing = activeTiming;
    return timing == null ? 1.0 : timing.rateAt(elapsedSeconds);
  }

  /** Starts executing `trajectoryValue` as a fresh, un-retimed plan. */
  function setActive(trajectoryValue:JointTrajectory):Void {
    if (activeTrajectory != null && activeTrajectory != trajectoryValue)
      discardNativeTrajectory(activeTrajectory);
    activeTrajectory = trajectoryValue;
    activeSource = trajectoryValue;
    activeTiming = null;
    activeJogAxis = null;
    resetExecutionState();
  }

  /** Swaps in a re-timed execution of the current plan. */
  function retimeActive(timing:TimeScaledTrajectory):Void {
    activeTrajectory = timing.trajectory;
    activeTiming = timing;
    resetExecutionState();
  }

  function resetExecutionState():Void {
    elapsedSeconds = 0.0;
    trajectorySubmitted = false;
    hostStopping = false;
    resetTrajectoryChunkState();
    trajectoryChunkReferences = new Map();
    trajectoryFinalTag = Int64.ofInt(0);
    trajectoryFinalEndSeconds = 0.0;
  }

  function planAxesFrom(start:Array<Float>, targets:Array<AxisTarget>,
      options:Null<MotionOptions>):JointTrajectory {
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
    return planLogical(start, movedAxes, logicalGoal, resolveLimits(targets, chosenOptions));
  }

  /**
   * Plans in logical axis units and maps each sample onto the joints. The
   * limits are the axes' authored units, and every joint of an axis follows
   * that axis's one profile, so the motors of a geared or dual-motor axis stay
   * in proportion throughout the move. Joints of other axes keep `start`.
   */
  function planLogical(start:Array<Float>, movedAxes:Array<MotionAxis>, logicalGoal:Array<Float>,
      limits:MotionLimits):JointTrajectory {
    if (robot.capabilities().supportsExecutionPlans) {
      var target = start.copy();
      for (index in 0...movedAxes.length)
        movedAxes[index].writeLogicalPosition(target, logicalGoal[index]);
      var stationary = true;
      for (joint in 0...start.length)
        if (Math.abs(target[joint] - start[joint]) > 1e-12) stationary = false;
      if (stationary)
        return new JointTrajectory([new JointTrajectorySample(0.0, start,
          [for (_ in start) 0.0])]);
      return nativeLogicalPlan(start, [for (_ in start) 0.0],
        [for (_ in start) 0.0], target, limits);
    }
    var logicalStart = [for (axisValue in movedAxes) axisValue.logicalPosition(start)];
    var logical = planner.plan(logicalStart, logicalGoal, limits);
    var samples:Array<JointTrajectorySample> = [];
    for (sample in logical.samples) {
      var positions = start.copy();
      var velocities = [for (_ in start) 0.0];
      var accelerations = [for (_ in start) 0.0];
      for (index in 0...movedAxes.length) {
        var axisValue = movedAxes[index];
        axisValue.writeLogicalPosition(positions, Math.max(axisValue.lowerLimit,
          Math.min(axisValue.upperLimit, sample.positions[index])));
        axisValue.writeLogicalDelta(velocities, sample.velocities[index]);
        axisValue.writeLogicalDelta(accelerations, sample.accelerations[index]);
      }
      samples.push(new JointTrajectorySample(sample.timeSeconds, positions, velocities,
        accelerations));
    }
    return new JointTrajectory(samples);
  }

  function nativeLogicalPlan(start:Array<Float>, velocity:Array<Float>,
      acceleration:Array<Float>, target:Array<Float>, limits:MotionLimits):JointTrajectory {
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
    var samples:Array<JointTrajectorySample> = [];
    var duration = native.durationSeconds();
    var count = Std.int(Math.max(1.0, Math.ceil(duration / fixedTimestepSeconds)));
    for (index in 0...(count + 1)) {
      var time = index == count ? duration : index * fixedTimestepSeconds;
      var state = native.evaluate(time);
      samples.push(new JointTrajectorySample(time, state.positions,
        state.velocities, state.accelerations));
    }
    var result = new JointTrajectory(samples);
    nativeTrajectories.push({trajectory: result, native: native});
    return result;
  }

  function planningStartPositions():Array<Float> {
    if (plannedEndPositions != null) return plannedEndPositions.copy();
    return robot.snapshot().positions.toArray();
  }

  function beginImmediate(trajectoryValue:JointTrajectory):Void {
    setActive(trajectoryValue);
    bufferedTotalSeconds = trajectoryValue.durationSeconds;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(trajectoryValue);
    resumeRequested = false;
    if (trajectoryValue.durationSeconds <= 0.0) {
      if (usesTrajectoryChunks(trajectoryValue))
        submitStationaryPlan(trajectoryValue.samples[0].positions);
      return;
    }
    if (usesTrajectoryChunks(trajectoryValue)) {
      fillNativeWindow();
    }
  }

  function enqueueTrajectory(trajectoryValue:JointTrajectory):Void {
    if (activeTrajectory == null && queuedTrajectories.length == 0) {
      bufferedTotalSeconds = 0.0;
      bufferedCompletedSeconds = 0.0;
    }
    queuedTrajectories.push(trajectoryValue);
    bufferedTotalSeconds += trajectoryValue.durationSeconds;
    plannedEndPositions = trajectoryEnd(trajectoryValue);
    activateNextTrajectory();
  }

  function activateNextTrajectory():Void {
    if (held || stoppingForReplacement || activeTrajectory != null ||
        queuedTrajectories.length == 0)
      return;
    var next:JointTrajectory = queuedTrajectories.shift();
    setActive(next);
    resumeRequested = false;
    if (next.durationSeconds <= 0.0) {
      if (usesTrajectoryChunks(next))
        submitStationaryPlan(next.samples[0].positions);
      return;
    }
    if (usesTrajectoryChunks(activeTrajectory)) fillNativeWindow();
  }

  function clearBufferedMotion():Void {
    if (activeTrajectory != null) discardNativeTrajectory(activeTrajectory);
    for (queued in queuedTrajectories) discardNativeTrajectory(queued);
    activeTrajectory = null;
    activeSource = null;
    activeTiming = null;
    queuedTrajectories = [];
    held = false;
    stoppingForReplacement = false;
    afterStop = [];
    activeJogAxis = null;
    bufferedTotalSeconds = 0.0;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = null;
    resumeRequested = false;
    nativeRefillDeferred = false;
    resetExecutionState();
  }

  function usesTrajectoryChunks(trajectoryValue:JointTrajectory):Bool {
    if (trajectoryValue == null) return false;
    return robot.capabilities().supportsExecutionPlans &&
      trajectoryValue.jointCount <= RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_JOINTS;
  }

  function submitActiveTrajectoryChunk():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    var smooth = nativeFor(trajectoryValue);
    var nativeSegments = smooth == null ? null : smooth.segments();
    if (nativeSegments != null && nativeSegments.length > 0 &&
        nativeSegments[0].coefficients[0].length >= 3) {
      if (trajectorySubmitted) return;
      var tag = nextTrajectoryTag;
      nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
      var segments = [for (segment in nativeSegments)
        new TrajectorySegment(segment.timeFromStartNs, segment.durationNs,
          segment.coefficients)];
      if (segments.length > 128) throw "Smooth plan exceeds one runtime submission";
      var start = smooth.evaluate(0.0);
      robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
        modelRevision, calibrationRevision, 1, start.positions, start.velocities,
        start.accelerations, segments)));
      trajectorySubmitted = true;
      trajectoryNextSampleIndex = trajectoryValue.samples.length - 1;
      trajectoryChunkStartSeconds = 0.0;
      trajectoryChunkEndSeconds = trajectoryValue.durationSeconds;
      trajectoryFinalTag = tag;
      trajectoryFinalEndSeconds = trajectoryValue.durationSeconds;
      trajectoryChunkReferences.set(Int64.toStr(tag),
        new TrajectoryChunkReference(trajectoryValue, 0.0));
      return;
    }
    var startIndex = trajectoryNextSampleIndex;
    var startTime = trajectoryValue.samples[startIndex].timeSeconds;
    var availableSegments = 4096 - robot.snapshot().trajectoryQueueDepth;
    var pointLimit = Std.int(Math.min(129, availableSegments + 1));
    if (pointLimit < 2) return;
    var endIndex = startIndex;
    var maximumEnd:Int = Std.int(Math.min(trajectoryValue.samples.length - 1,
      startIndex + pointLimit - 1));
    for (index in (startIndex + 1)...(maximumEnd + 1)) {
      var sample = trajectoryValue.samples[index];
      endIndex = index;
    }
    var tag = nextTrajectoryTag;
    nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
    var times = [for (index in startIndex...(endIndex + 1))
      trajectoryValue.samples[index].timeSeconds - startTime];
    var positions = [for (index in startIndex...(endIndex + 1))
      trajectoryValue.samples[index].positions];
    var segments:Array<TrajectorySegment> = [];
    if (nativeSegments != null &&
        nativeSegments.length == trajectoryValue.samples.length - 1) {
      var nativeStartNs = nativeSegments[startIndex].timeFromStartNs;
      for (index in startIndex...endIndex) {
        var segment = nativeSegments[index];
        segments.push(new TrajectorySegment(
          Int64.sub(segment.timeFromStartNs, nativeStartNs),
          segment.durationNs, segment.coefficients));
      }
    } else {
      var native = Trajectory.fromPositionSamples(times, positions);
      for (segment in native.segments())
        segments.push(new TrajectorySegment(segment.timeFromStartNs,
          segment.durationNs, segment.coefficients));
      native.dispose();
    }
    var startVelocity = [for (_ in positions[0]) 0.0];
    if (startIndex > 0) {
      var before = trajectoryValue.samples[startIndex - 1];
      var after = trajectoryValue.samples[startIndex];
      var seconds = after.timeSeconds - before.timeSeconds;
      for (joint in 0...startVelocity.length)
        startVelocity[joint] = (after.positions[joint] - before.positions[joint]) / seconds;
    }
    try robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
      modelRevision, calibrationRevision, 1, positions[0], startVelocity,
      [for (_ in positions[0]) 0.0], segments, null, null, null, null, null,
      endIndex >= trajectoryValue.samples.length - 1))) catch (error:Dynamic)
      throw 'plan chunk [$startIndex,$endIndex] of ${trajectoryValue.samples.length}, '
        + 'start=${positions[0]}, end=${positions[positions.length - 1]}, '
        + 'segments=${segments.length}, firstT=${segments[0].timeFromStartNs}, '
        + 'lastT=${segments[segments.length - 1].timeFromStartNs}: $error';
    trajectorySubmitted = true;
    trajectoryNextSampleIndex = endIndex;
    trajectoryChunkStartSeconds = startTime;
    trajectoryChunkEndSeconds = trajectoryValue.samples[endIndex].timeSeconds;
    trajectoryChunkReferences.set(Int64.toStr(tag),
      new TrajectoryChunkReference(trajectoryValue, startTime));
    if (endIndex >= trajectoryValue.samples.length - 1) {
      trajectoryFinalTag = tag;
      trajectoryFinalEndSeconds = trajectoryChunkEndSeconds;
    }
  }

  /** Keep a fixed execution window queued so native HOLD never needs a host lead estimate. */
  function fillNativeWindow():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !usesTrajectoryChunks(trajectoryValue)) return;
    while (trajectoryNextSampleIndex < trajectoryValue.samples.length - 1 &&
        trajectoryChunkEndSeconds - elapsedSeconds < 2.0) {
      var before = trajectoryNextSampleIndex;
      submitActiveTrajectoryChunk();
      if (trajectoryNextSampleIndex == before) break;
    }
  }

  function submitStationaryPlan(positions:Array<Float>):Void {
    var tag = nextTrajectoryTag;
    nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
    var zeros = [for (_ in positions) 0.0];
    robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
      modelRevision, calibrationRevision, 1, positions, zeros, zeros,
      [new TrajectorySegment(Int64.ofInt(0),
        secondsToNanoseconds(fixedTimestepSeconds), [for (value in positions) [value]])])));
  }

  function syncFromRuntime():RobotSnapshot {
    var observation = robot.snapshot();
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !usesTrajectoryChunks(trajectoryValue)) return observation;
    var reference = trajectoryChunkReferences.get(Int64.toStr(observation.trajectoryTag));
    if (reference != null && reference.trajectory == trajectoryValue) {
      var runtimeSeconds = Int64.toFloat(observation.trajectoryTagTimeNs) /
        1000000000.0;
      if (Math.isFinite(runtimeSeconds))
        elapsedSeconds = Math.min(trajectoryValue.durationSeconds,
          Math.max(0.0, reference.startSeconds + runtimeSeconds));
    }
    return observation;
  }

  function trajectoryFinishedInRuntime(observation:RobotSnapshot):Bool {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !trajectorySubmitted ||
        Int64.compare(trajectoryFinalTag, Int64.ofInt(0)) == 0)
      return false;
    if (trajectoryFinalEndSeconds < trajectoryValue.durationSeconds - 1e-9)
      return false;
    if (Int64.compare(observation.trajectoryTag, trajectoryFinalTag) != 0)
      return false;
    var finalTime = Int64.toFloat(observation.trajectoryTagTimeNs) /
      1000000000.0;
    var reachedEnd = Math.isFinite(finalTime) &&
      trajectoryFinalEndSeconds - trajectoryChunkStartSeconds <= finalTime + 1e-9;
    return reachedEnd && !observation.trajectoryActive &&
      observation.trajectoryQueueDepth == 0;
  }

  function completeActiveTrajectory():Void {
    var source = activeSource;
    if (activeTrajectory == null || source == null) return;
    discardNativeTrajectory(activeTrajectory);
    bufferedCompletedSeconds += source.durationSeconds;
    activeTrajectory = null;
    activeSource = null;
    activeTiming = null;
    resetExecutionState();
    activateNextTrajectory();
  }

  function discardNativeTrajectory(value:JointTrajectory):Void {
    for (index in 0...nativeTrajectories.length)
      if (nativeTrajectories[index].trajectory == value) {
        nativeTrajectories[index].native.dispose();
        nativeTrajectories.splice(index, 1);
        return;
      }
  }

  function nativeFor(value:JointTrajectory):Null<Trajectory> {
    for (entry in nativeTrajectories)
      if (entry.trajectory == value) return entry.native;
    return null;
  }

  function shouldRefillTrajectory(dt:Float):Bool {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || trajectoryNextSampleIndex >= trajectoryValue.samples.length - 1)
      return false;
    var lead = Math.max(2.0, Math.max(fixedTimestepSeconds, dt) * 2.0);
    if (trajectoryChunkEndSeconds - elapsedSeconds <= lead + 1e-9) return true;
    // If an owner cycle drained the native window before the host update,
    // refill immediately from the deterministic source trajectory.
    // At t=0 the first chunk may still be in the runtime mailbox, so its
    // snapshot quite correctly reports an empty queue until the owner cycle
    // accepts that command.
    if (elapsedSeconds <= 1e-9) return false;
    var observation = robot.snapshot();
    return !observation.trajectoryActive || observation.trajectoryQueueDepth == 0;
  }

  function resetTrajectoryChunkState():Void {
    trajectoryNextSampleIndex = 0;
    trajectoryChunkEndSeconds = 0.0;
    trajectoryChunkStartSeconds = 0.0;
  }

  static function secondsToNanoseconds(seconds:Float):Int64 {
    return Trajectory.nanoseconds(seconds);
  }

  static function trajectoryEnd(trajectoryValue:JointTrajectory):Array<Float>
    return trajectoryValue.samples[trajectoryValue.samples.length - 1].positions.copy();

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

private class TrajectoryChunkReference {
  public final trajectory:JointTrajectory;
  public final startSeconds:Float;

  public function new(trajectory:JointTrajectory, startSeconds:Float) {
    this.trajectory = trajectory;
    this.startSeconds = startSeconds;
  }
}

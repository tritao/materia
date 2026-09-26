package motionkit;

import haxe.Int64;
import motionkit.Feed;
import motionkit.axis.MotionAxis;
import motionkit.axis.MotionSystemBlueprint;
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
import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.StopMode;
import robotkit.world.TrajectoryChunk;
import robotkit.world.TrajectoryPoint;

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
  public final planner:TrajectoryPlanner;
  public final linePlanner:LineLookaheadPlanner;
  /** Trajectory being executed; a re-timed copy of activeSource after a hold or resume. */
  var activeTrajectory:Null<JointTrajectory> = null;
  /** The planned trajectory behind activeTrajectory. */
  var activeSource:Null<JointTrajectory> = null;
  /** Maps activeTrajectory time back to activeSource time when it was re-timed. */
  var activeTiming:Null<TimeScaledTrajectory> = null;
  var queuedTrajectories:Array<JointTrajectory> = [];
  var elapsedSeconds:Float = 0.0;
  var held:Bool = false;
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
  /** Splice point for the next submitted chunk; a zero tag appends. */
  var nextChunkSpliceTag:Int64 = Int64.ofInt(0);
  var nextChunkSpliceTimeNs:Int64 = Int64.ofInt(0);
  /** A jog splice the runtime has not yet taken over, with how to recover if dropped. */
  var pendingSplice:Null<PendingSplice> = null;
  /** Per-joint acceleration limits from the logical axes; zero is unconstrained. */
  final jointAccelerationLimits:Array<Float>;

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
    beginStop();
  }

  /**
   * Resumes a held buffer from where it stopped, speeding back up along the
   * path. If the hold is still slowing down, it resumes once at rest.
   */
  public function resume():Void {
    if (!held) return;
    resumeRequested = true;
    advanceStop();
  }

  /**
   * Aborts buffered motion. A normal abort slows to rest along the current
   * path before discarding it; an emergency stop acts immediately.
   */
  public function abort(?mode:StopMode = StopMode.Normal):Void {
    if (mode == StopMode.Normal && isMotionInProgress()) {
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
      // The runtime stops by slowing along the queued path, which takes at
      // most v/a of trajectory time. A stop can arrive when less than that is
      // queued, so top up the queue before the stop command. Should the queue
      // still run out, the runtime finishes on a limited ramp.
      syncFromRuntime();
      refillTrajectoryForHold();
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
    var mapped:Array<JointTrajectorySample> = [];
    for (sample in cartesian.samples) {
      var positions = start.copy();
      var velocities = [for (_ in start) 0.0];
      var accelerations = [for (_ in start) 0.0];
      xAxis.writeLogicalPosition(positions, sample.positions[0]);
      yAxis.writeLogicalPosition(positions, sample.positions[1]);
      zAxis.writeLogicalPosition(positions, sample.positions[2]);
      writeLogicalVector(xAxis, velocities, sample.velocities[0]);
      writeLogicalVector(yAxis, velocities, sample.velocities[1]);
      writeLogicalVector(zAxis, velocities, sample.velocities[2]);
      writeLogicalVector(xAxis, accelerations, sample.accelerations[0]);
      writeLogicalVector(yAxis, accelerations, sample.accelerations[1]);
      writeLogicalVector(zAxis, accelerations, sample.accelerations[2]);
      mapped.push(new JointTrajectorySample(sample.timeSeconds, positions, velocities,
        accelerations));
    }
    return new JointTrajectory(mapped);
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
   * Changes a running jog on the same axis without stopping: plans from the
   * jog's position and velocity a few periods ahead and splices the new
   * profile in there, so speed and direction change within the acceleration
   * limit. Returns null when the jog cannot be continued this way.
   */
  function continueJog(axisValue:MotionAxis, velocity:Float, durationSeconds:Float,
      acceleration:Float):Null<JointTrajectory> {
    var executing = activeTrajectory;
    if (executing == null || activeJogAxis != axisValue.id || held || stoppingForReplacement ||
        hostStopping || afterStop.length > 0 || queuedTrajectories.length > 0 ||
        pendingSplice != null)
      return null;
    var buffered = usesTrajectoryChunks(executing);
    var spliceSeconds = elapsedSeconds;
    var spliceTag = Int64.ofInt(0);
    var spliceTimeNs = Int64.ofInt(0);
    if (buffered) {
      var observation = syncFromRuntime();
      if (!observation.trajectoryActive) return null;
      var reference = trajectoryChunkReferences.get(Int64.toStr(observation.trajectoryTag));
      if (reference == null || reference.trajectory != executing) return null;
      // Leave a few periods for the command to reach the runtime before the
      // splice point; a splice that still arrives late is dropped safely.
      var tagSeconds = Std.parseFloat(Int64.toStr(observation.trajectoryTagTimeNs)) / 1000000000.0 +
        3.0 * fixedTimestepSeconds;
      spliceSeconds = reference.startSeconds + tagSeconds;
      if (spliceSeconds >= trajectoryChunkEndSeconds) return null;
      spliceTag = observation.trajectoryTag;
      spliceTimeNs = secondsToNanoseconds(tagSeconds);
    }
    if (spliceSeconds >= executing.durationSeconds - fixedTimestepSeconds) return null;

    var start = executing.sample(spliceSeconds).positions;
    var startLogical = axisValue.logicalPosition(start);
    var startVelocity = logicalVelocityAt(executing, axisValue, spliceSeconds);
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
    if (buffered) {
      nextChunkSpliceTag = spliceTag;
      nextChunkSpliceTimeNs = spliceTimeNs;
      submitActiveTrajectoryChunk();
      pendingSplice = new PendingSplice(spliceTag, spliceTimeNs, () -> {
        activeJogAxis = null;
        jog(axisValue.id, velocity, durationSeconds, acceleration);
      });
    }
    return trajectoryValue;
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
   * Confirms a pending jog splice once the runtime runs the new profile, or
   * recovers when the runtime dropped it for arriving late: the old jog is
   * still running, so stop along it and start the requested jog from rest.
   */
  function checkPendingSplice(observation:RobotSnapshot):Void {
    var pending = pendingSplice;
    if (pending == null) return;
    if (trajectoryChunkReferences.exists(Int64.toStr(observation.trajectoryTag))) {
      pendingSplice = null;
      return;
    }
    var passed = Int64.compare(observation.trajectoryTag, pending.oldTag) == 0 &&
      Int64.compare(observation.trajectoryTagTimeNs, pending.spliceTimeNs) >= 0;
    if (!passed && observation.trajectoryActive) return;
    pendingSplice = null;
    queuedTrajectories = [];
    afterStop = [() -> {
      clearBufferedMotion();
      pending.retry();
    }];
    held = false;
    resumeRequested = false;
    stoppingForReplacement = true;
    robot.stop(StopMode.Normal);
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
          syncFromRuntime();
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
      checkPendingSplice(observation);
      if (stoppingForReplacement) return true;
      if (trajectoryFinishedInRuntime(observation)) {
        completeActiveTrajectory();
        return activeTrajectory != null;
      }
      if (!trajectorySubmitted || shouldRefillTrajectory(dt)) submitActiveTrajectoryChunk();
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
    if (usesTrajectoryChunks(trajectoryValue)) {
      // A direct move replaces the native runtime's queue as well as this
      // local buffer. The position batch is an ordered flush marker; the
      // following chunk then starts from the current observed pose instead of
      // being appended behind stale motion.
      replaceRuntimeMotion();
      submitActiveTrajectoryChunk();
    }
  }

  function replaceRuntimeMotion():Void {
    var positions = robot.snapshot().positions.toArray();
    var targets:Array<JointTarget> = [];
    for (joint in 0...positions.length)
      targets.push(JointTarget.position(joint, positions[joint]));
    robot.submit(RobotCommand.JointTargets(targets, null));
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
    setActive(queuedTrajectories.shift());
    resumeRequested = false;
    if (usesTrajectoryChunks(activeTrajectory)) submitActiveTrajectoryChunk();
  }

  function clearBufferedMotion():Void {
    activeTrajectory = null;
    activeSource = null;
    activeTiming = null;
    queuedTrajectories = [];
    held = false;
    stoppingForReplacement = false;
    afterStop = [];
    pendingSplice = null;
    activeJogAxis = null;
    bufferedTotalSeconds = 0.0;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = null;
    resumeRequested = false;
    resetExecutionState();
  }

  function usesTrajectoryChunks(trajectoryValue:JointTrajectory):Bool {
    if (trajectoryValue == null) return false;
    return robot.capabilities().supportsTrajectoryQueue &&
      trajectoryValue.jointCount <= TrajectoryPoint.MAX_JOINTS;
  }

  function submitActiveTrajectoryChunk():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    var startIndex = trajectoryNextSampleIndex;
    var startTime = trajectoryValue.samples[startIndex].timeSeconds;
    var points:Array<TrajectoryPoint> = [];
    points.push(new TrajectoryPoint(Int64.ofInt(0), trajectoryValue.samples[startIndex].positions));
    var pointLimit = TrajectoryChunk.MAX_POINTS;
    if (startIndex > 0) {
      var observation = robot.snapshot();
      if (observation.trajectoryActive && observation.trajectoryQueueDepth > 0)
        pointLimit = Std.int(Math.max(0,
          TrajectoryChunk.MAX_POINTS - observation.trajectoryQueueDepth));
    }
    if (pointLimit < 2) return;
    var endIndex = startIndex;
    var maximumEnd:Int = Std.int(Math.min(trajectoryValue.samples.length - 1,
      startIndex + pointLimit - 1));
    for (index in (startIndex + 1)...(maximumEnd + 1)) {
      var sample = trajectoryValue.samples[index];
      points.push(new TrajectoryPoint(secondsToNanoseconds(sample.timeSeconds - startTime),
        sample.positions));
      endIndex = index;
    }
    var tag = nextTrajectoryTag;
    nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
    robot.submit(RobotCommand.TrajectoryChunk(new TrajectoryChunk(points, tag,
      nextChunkSpliceTag, nextChunkSpliceTimeNs)));
    nextChunkSpliceTag = Int64.ofInt(0);
    nextChunkSpliceTimeNs = Int64.ofInt(0);
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

  function syncFromRuntime():RobotSnapshot {
    var observation = robot.snapshot();
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !usesTrajectoryChunks(trajectoryValue)) return observation;
    var reference = trajectoryChunkReferences.get(Int64.toStr(observation.trajectoryTag));
    if (reference != null && reference.trajectory == trajectoryValue) {
      var runtimeSeconds = Std.parseFloat(Int64.toStr(observation.trajectoryTagTimeNs)) /
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
    var finalTime = Std.parseFloat(Int64.toStr(observation.trajectoryTagTimeNs)) /
      1000000000.0;
    var reachedEnd = Math.isFinite(finalTime) &&
      trajectoryFinalEndSeconds - trajectoryChunkStartSeconds <= finalTime + 1e-9;
    return reachedEnd && !observation.trajectoryActive &&
      observation.trajectoryQueueDepth == 0;
  }

  function completeActiveTrajectory():Void {
    var source = activeSource;
    if (activeTrajectory == null || source == null) return;
    bufferedCompletedSeconds += source.durationSeconds;
    activeTrajectory = null;
    activeSource = null;
    activeTiming = null;
    resetExecutionState();
    activateNextTrajectory();
  }

  function shouldRefillTrajectory(dt:Float):Bool {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || trajectoryNextSampleIndex >= trajectoryValue.samples.length - 1)
      return false;
    var lead = Math.max(fixedTimestepSeconds, dt) * 2.0;
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

  /**
   * Keeps enough source trajectory queued for the runtime's path-following
   * stop. Used at the hold boundary, where the ordinary two-tick streaming
   * lead may be shorter than v/a.
   */
  function refillTrajectoryForHold():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !usesTrajectoryChunks(trajectoryValue)) return;
    var requiredLead = holdStopLeadSeconds();
    var attempts = 0;
    while (trajectoryNextSampleIndex < trajectoryValue.samples.length - 1 &&
        trajectoryChunkEndSeconds - elapsedSeconds < requiredLead - 1e-9 &&
        attempts < 8) {
      var previousEnd = trajectoryChunkEndSeconds;
      submitActiveTrajectoryChunk();
      attempts += 1;
      if (trajectoryChunkEndSeconds <= previousEnd + 1e-9) break;
    }
  }

  function holdStopLeadSeconds():Float {
    var result = fixedTimestepSeconds * 2.0;
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return result;
    var current = trajectoryValue.sample(elapsedSeconds);
    var nextTime = Math.min(trajectoryValue.durationSeconds,
      elapsedSeconds + Math.max(fixedTimestepSeconds, 1e-6));
    var next = trajectoryValue.sample(nextTime);
    for (joint in 0...jointAccelerationLimits.length) {
      var limit = jointAccelerationLimits[joint];
      if (limit <= 0.0 || joint >= current.positions.length) continue;
      var sampledVelocity = Math.abs(current.velocities[joint]);
      var finiteDifference = nextTime > elapsedSeconds
        ? Math.abs(next.positions[joint] - current.positions[joint]) /
          (nextTime - elapsedSeconds) : 0.0;
      result = Math.max(result, Math.max(sampledVelocity, finiteDifference) / limit);
    }
    // Leave two owner periods of margin for mailbox and simulation phase
    // ordering. The runtime itself enforces the acceleration bound.
    return result + fixedTimestepSeconds * 2.0;
  }

  function resetTrajectoryChunkState():Void {
    trajectoryNextSampleIndex = 0;
    trajectoryChunkEndSeconds = 0.0;
    trajectoryChunkStartSeconds = 0.0;
  }

  static function secondsToNanoseconds(seconds:Float):Int64 {
    if (!Math.isFinite(seconds) || seconds < 0.0)
      throw "Trajectory timestamp must be finite and non-negative";
    // Int64.fromFloat and Math.round follow the host Int range on some Haxe
    // targets. Parse the integral decimal representation so windows longer
    // than 2.147 s do not wrap before they reach the native trajectory ABI.
    // Positive truncation after adding 0.5 implements rounding without the
    // Haxe Math.round/floor helpers, whose return type is the host Int.
    var value:Float = seconds * 1000000000.0 + 0.5;
    var high = Std.int(value / 4294967296.0);
    var lowValue = value - high * 4294967296.0;
    var low = lowValue >= 2147483648.0
      ? Std.int(lowValue - 4294967296.0) : Std.int(lowValue);
    return Int64.make(high, low);
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

private class PendingSplice {
  public final oldTag:Int64;
  public final spliceTimeNs:Int64;
  public final retry:Void -> Void;

  public function new(oldTag:Int64, spliceTimeNs:Int64, retry:Void -> Void) {
    this.oldTag = oldTag;
    this.spliceTimeNs = spliceTimeNs;
    this.retry = retry;
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

package motionkit;

import haxe.Int64;
import motionkit.Feed;
import motionkit.axis.MotionAxis;
import motionkit.axis.MotionSystemBlueprint;
import motionkit.path.PathPoint;
import motionkit.path.GeometricPath;
import motionkit.planner.LineLookaheadPlanner;
import motionkit.planner.PathPlanningOptions;
import motionkit.planner.TrajectoryPlanner;
import motionkit.planner.TrapezoidalPlanner;
import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;
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
 */
class MotionSystem {
  public final robot:Robot;
  public final axes:Array<MotionAxis>;
  public final fixedTimestepSeconds:Float;
  public final planner:TrajectoryPlanner;
  public final linePlanner:LineLookaheadPlanner;
  var activeTrajectory:Null<JointTrajectory> = null;
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
  var trajectoryChunkInitialPositions:Null<Array<Float>> = null;
  var nextTrajectoryTag:Int64 = Int64.ofInt(1);
  var trajectoryChunkReferences:Map<String, TrajectoryChunkReference> = new Map();
  var trajectoryFinalTag:Int64 = Int64.ofInt(0);
  var trajectoryFinalEndSeconds:Float = 0.0;
  var resumeRequested:Bool = false;

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
  }

  public function axis(id:String):Null<MotionAxis> {
    for (value in axes) if (value.id == id) return value;
    return null;
  }

  public function isMoving():Bool return activeTrajectory != null;

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
    var completed = bufferedCompletedSeconds +
      (activeTrajectory == null ? 0.0 : Math.min(activeTrajectory.durationSeconds, elapsedSeconds));
    return Math.min(1.0, Math.max(0.0, completed / bufferedTotalSeconds));
  }

  public function isHolding():Bool return held;

  /** Plans a coordinated move while leaving unspecified axes at their current positions. */
  public function moveAxes(targets:Array<AxisTarget>, ?options:MotionOptions):JointTrajectory {
    var trajectoryValue = planAxesFrom(robot.snapshot().positions.toArray(), targets, options);
    clearBufferedMotion();
    beginImmediate(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds a coordinated axis move behind all motion already in the buffer. */
  public function queueAxes(targets:Array<AxisTarget>, ?options:MotionOptions):JointTrajectory {
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
    enqueueTrajectory(trajectoryValue);
  }

  /** Plans a straight Cartesian move for a direct XYZ gantry. */
  public function moveLinear(target:PathPoint, feed:Feed,
      ?options:MotionOptions):JointTrajectory {
    var trajectoryValue = planLinearPathFrom(robot.snapshot().positions.toArray(), target,
      feed, options);
    clearBufferedMotion();
    beginImmediate(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds a straight Cartesian move behind all motion already in the buffer. */
  public function queueLinear(target:PathPoint, feed:Feed,
      ?options:MotionOptions):JointTrajectory {
    var trajectoryValue = planLinearPathFrom(planningStartPositions(), target, feed, options);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Plans a connected Cartesian polyline for a direct XYZ machine. */
  public function movePath(path:GeometricPath, ?pathOptions:PathPlanningOptions,
      ?motionOptions:MotionOptions):JointTrajectory {
    var trajectoryValue = planPathFrom(robot.snapshot().positions.toArray(), path,
      pathOptions, motionOptions);
    clearBufferedMotion();
    beginImmediate(trajectoryValue);
    return trajectoryValue;
  }

  /** Adds a connected Cartesian polyline behind motion already in the buffer. */
  public function queuePath(path:GeometricPath, ?pathOptions:PathPlanningOptions,
      ?motionOptions:MotionOptions):JointTrajectory {
    var trajectoryValue = planPathFrom(planningStartPositions(), path,
      pathOptions, motionOptions);
    enqueueTrajectory(trajectoryValue);
    return trajectoryValue;
  }

  /** Holds buffered motion and requests a controlled stop without discarding it. */
  public function hold():Void {
    if (held) return;
    resumeRequested = false;
    if (activeTrajectory != null) {
      if (usesTrajectoryChunks(activeTrajectory)) {
        // The runtime stops by slowing along the queued path, which takes at
        // most v/a of trajectory time. A hold can arrive when less than that
        // is queued, so top up the queue before the stop command. Should the
        // queue still run out, the runtime finishes on a limited ramp.
        syncFromRuntime();
        refillTrajectoryForHold();
      }
      robot.stop(StopMode.Normal);
    }
    held = true;
  }

  /** Resumes a held buffer at its current deterministic trajectory time. */
  public function resume():Void {
    if (!held) return;
    resumeRequested = true;
    tryResume();
  }

  /** Aborts buffered motion and separates a controlled stop from emergency stop. */
  public function abort(?mode:StopMode = StopMode.Normal):Void {
    clearBufferedMotion();
    robot.stop(mode);
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
  public function home(?options:MotionOptions):JointTrajectory {
    return moveAxes([for (axisValue in axes) new AxisTarget(axisValue.id, axisValue.homePosition)], options);
  }

  /**
   * Plans a bounded timed jog in one logical axis. The endpoint is clamped to
   * the authored axis limits, while all physical joints in a coordinated axis
   * group receive the same logical displacement and scaled velocity.
   */
  public function jog(axisId:String, velocity:Float, durationSeconds:Float,
      ?maxAcceleration:Float):JointTrajectory {
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

    var start = robot.snapshot().positions.toArray();
    var startLogical = axisValue.logicalPosition(start);
    var requestedEnd = startLogical + velocity * durationSeconds;
    var endLogical = Math.max(axisValue.lowerLimit,
      Math.min(axisValue.upperLimit, requestedEnd));
    var end = start.copy();
    axisValue.writeLogicalPosition(end, endLogical);
    var trajectoryValue = planner.plan(start, end,
      new MotionLimits(Math.abs(velocity), acceleration));
    clearBufferedMotion();
    beginImmediate(trajectoryValue);
    return trajectoryValue;
  }

  /**
   * Advances the local deterministic clock and submits one complete position
   * batch. Pass no duration to use the compiled fixed timestep.
   */
  public function update(?dtSeconds:Float = -1.0):Bool {
    if (held) {
      // No refills while held: hold() already queued enough path for the
      // whole stop, and a late chunk could otherwise land after the stop
      // finished and be taken as a resume.
      if (activeTrajectory != null && usesTrajectoryChunks(activeTrajectory))
        syncFromRuntime();
      if (resumeRequested) tryResume();
      if (held) return false;
    }
    if (activeTrajectory == null) activateNextTrajectory();
    if (activeTrajectory == null) return false;
    var dt = dtSeconds < 0.0 ? fixedTimestepSeconds : dtSeconds;
    if (!Math.isFinite(dt) || dt <= 0.0) throw "Motion-system update duration must be finite and positive";
    var trajectory = activeTrajectory;
    if (trajectory.durationSeconds <= 0.0) {
      completeActiveTrajectory();
      return activeTrajectory != null;
    }
    if (usesTrajectoryChunks(trajectory)) {
      var observation = syncFromRuntime();
      if (trajectoryFinishedInRuntime(observation)) {
        completeActiveTrajectory();
        return activeTrajectory != null;
      }
      if (!trajectorySubmitted || shouldRefillTrajectory(dt)) submitActiveTrajectoryChunk();
    } else {
      var sample = trajectory.sample(elapsedSeconds);
      var targets:Array<JointTarget> = [];
      for (i in 0...sample.positions.length)
        targets.push(JointTarget.position(i, sample.positions[i]));
      robot.submit(RobotCommand.JointTargets(targets, null));
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

  function planAxesFrom(start:Array<Float>, targets:Array<AxisTarget>,
      options:Null<MotionOptions>):JointTrajectory {
    if (targets == null || targets.length == 0) throw "Axis move needs at least one target";
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
    return planner.plan(start, goal, resolveLimits(targets, chosenOptions));
  }

  function planningStartPositions():Array<Float> {
    if (plannedEndPositions != null) return plannedEndPositions.copy();
    return robot.snapshot().positions.toArray();
  }

  function beginImmediate(trajectoryValue:JointTrajectory):Void {
    activeTrajectory = trajectoryValue;
    elapsedSeconds = 0.0;
    trajectorySubmitted = false;
    bufferedTotalSeconds = trajectoryValue.durationSeconds;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(trajectoryValue);
    resumeRequested = false;
    resetTrajectoryChunkState();
    trajectoryChunkReferences = new Map();
    trajectoryFinalTag = Int64.ofInt(0);
    trajectoryFinalEndSeconds = 0.0;
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
    if (held || activeTrajectory != null || queuedTrajectories.length == 0) return;
    activeTrajectory = queuedTrajectories.shift();
    elapsedSeconds = 0.0;
    trajectorySubmitted = false;
    resumeRequested = false;
    resetTrajectoryChunkState();
    trajectoryChunkReferences = new Map();
    trajectoryFinalTag = Int64.ofInt(0);
    trajectoryFinalEndSeconds = 0.0;
    if (usesTrajectoryChunks(activeTrajectory)) submitActiveTrajectoryChunk();
  }

  function clearBufferedMotion():Void {
    activeTrajectory = null;
    queuedTrajectories = [];
    elapsedSeconds = 0.0;
    held = false;
    bufferedTotalSeconds = 0.0;
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = null;
    trajectorySubmitted = false;
    resumeRequested = false;
    resetTrajectoryChunkState();
    trajectoryChunkReferences = new Map();
    trajectoryFinalTag = Int64.ofInt(0);
    trajectoryFinalEndSeconds = 0.0;
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
    var resuming = trajectoryChunkInitialPositions != null;
    var startTime = resuming ? trajectoryChunkStartSeconds :
      (startIndex == 0 ? 0.0 : trajectoryValue.samples[startIndex].timeSeconds);
    var initialPositions = resuming ? trajectoryChunkInitialPositions :
      trajectoryValue.sample(startTime).positions;
    var points:Array<TrajectoryPoint> = [];
    points.push(new TrajectoryPoint(Int64.ofInt(0), cast initialPositions));
    var pointLimit = TrajectoryChunk.MAX_POINTS;
    if (startIndex > 0 || resuming) {
      var observation = robot.snapshot();
      if (observation.trajectoryActive && observation.trajectoryQueueDepth > 0)
        pointLimit = Std.int(Math.max(0,
          TrajectoryChunk.MAX_POINTS - observation.trajectoryQueueDepth));
    }
    if (pointLimit < 2) return;
    var endIndex = startIndex > trajectoryValue.samples.length - 1
      ? trajectoryValue.samples.length - 1 : startIndex;
    var maximumEnd:Int = Std.int(Math.min(trajectoryValue.samples.length - 1,
      startIndex + pointLimit - (resuming ? 2 : 1)));
    var firstFutureIndex = resuming ? startIndex : startIndex + 1;
    for (index in firstFutureIndex...(maximumEnd + 1)) {
      var sample = trajectoryValue.samples[index];
      points.push(new TrajectoryPoint(secondsToNanoseconds(sample.timeSeconds - startTime),
        sample.positions));
      endIndex = index;
    }
    var tag = nextTrajectoryTag;
    nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
    robot.submit(RobotCommand.TrajectoryChunk(new TrajectoryChunk(points, tag)));
    trajectorySubmitted = true;
    trajectoryChunkInitialPositions = null;
    trajectoryNextSampleIndex = endIndex;
    trajectoryChunkStartSeconds = startTime;
    trajectoryChunkEndSeconds = resuming && startIndex > trajectoryValue.samples.length - 1
      ? trajectoryValue.durationSeconds : trajectoryValue.samples[endIndex].timeSeconds;
    trajectoryChunkReferences.set(Int64.toStr(tag),
      new TrajectoryChunkReference(trajectoryValue, startTime));
    if (endIndex >= trajectoryValue.samples.length - 1) {
      trajectoryFinalTag = tag;
      trajectoryFinalEndSeconds = trajectoryChunkEndSeconds;
    }
  }

  function prepareTrajectoryResume():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    var nextIndex = 0;
    while (nextIndex < trajectoryValue.samples.length &&
        trajectoryValue.samples[nextIndex].timeSeconds <= elapsedSeconds + 1e-9)
      nextIndex++;
    trajectoryNextSampleIndex = nextIndex;
    trajectoryChunkStartSeconds = elapsedSeconds;
    trajectoryChunkInitialPositions = trajectoryValue.sample(elapsedSeconds).positions;
    trajectoryChunkEndSeconds = elapsedSeconds;
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

  function tryResume():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) {
      held = false;
      resumeRequested = false;
      activateNextTrajectory();
      return;
    }
    if (usesTrajectoryChunks(trajectoryValue)) {
      var observation = syncFromRuntime();
      if (observation.trajectoryActive) return;
      if (elapsedSeconds >= trajectoryValue.durationSeconds - 1e-9) {
        held = false;
        resumeRequested = false;
        completeActiveTrajectory();
        return;
      }
      prepareTrajectoryResume();
      trajectorySubmitted = false;
      held = false;
      resumeRequested = false;
      submitActiveTrajectoryChunk();
    } else {
      held = false;
      resumeRequested = false;
    }
    activateNextTrajectory();
  }

  function completeActiveTrajectory():Void {
    if (activeTrajectory == null) return;
    bufferedCompletedSeconds += activeTrajectory.durationSeconds;
    activeTrajectory = null;
    elapsedSeconds = 0.0;
    trajectorySubmitted = false;
    resetTrajectoryChunkState();
    trajectoryChunkReferences = new Map();
    trajectoryFinalTag = Int64.ofInt(0);
    trajectoryFinalEndSeconds = 0.0;
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
    var result = Math.max(fixedTimestepSeconds * 2.0, fixedTimestepSeconds);
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return result;
    var current = trajectoryValue.sample(elapsedSeconds);
    var nextTime = Math.min(trajectoryValue.durationSeconds,
      elapsedSeconds + Math.max(fixedTimestepSeconds, 1e-6));
    var next = trajectoryValue.sample(nextTime);
    for (axisValue in axes) {
      if (axisValue.maxAcceleration <= 0.0) continue;
      for (joint in axisValue.jointIndices) {
        if (joint >= current.positions.length || joint >= next.positions.length) continue;
        var sampledVelocity = joint < current.velocities.length
          ? Math.abs(current.velocities[joint]) : 0.0;
        var finiteDifference = nextTime > elapsedSeconds
          ? Math.abs(next.positions[joint] - current.positions[joint]) /
            (nextTime - elapsedSeconds) : 0.0;
        var velocity = Math.max(sampledVelocity, finiteDifference);
        result = Math.max(result, velocity / axisValue.maxAcceleration);
      }
    }
    // Leave two owner periods of margin for mailbox and simulation phase
    // ordering. The runtime itself enforces the acceleration bound.
    return result + fixedTimestepSeconds * 2.0;
  }

  function resetTrajectoryChunkState():Void {
    trajectoryNextSampleIndex = 0;
    trajectoryChunkEndSeconds = 0.0;
    trajectoryChunkStartSeconds = 0.0;
    trajectoryChunkInitialPositions = null;
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

private class TrajectoryChunkReference {
  public final trajectory:JointTrajectory;
  public final startSeconds:Float;

  public function new(trajectory:JointTrajectory, startSeconds:Float) {
    this.trajectory = trajectory;
    this.startSeconds = startSeconds;
  }
}

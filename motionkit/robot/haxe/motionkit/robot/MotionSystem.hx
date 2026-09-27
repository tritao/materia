package motionkit.robot;

import haxe.Int64;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.axis.MotionAxis;
import motionkit.path.PathPoint;
import motionkit.path.GeometricPath;
import motionkit.path.ArcSegment;
import motionkit.path.CornerBlender;
import motionkit.path.LineSegment;
import motionkit.path.QuinticBlend;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.ToppraPathTiming;
import motionkit.planner.PathPlanningOptions;
import motionkit.trajectory.MotionLimits;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.TrajectoryState;
import motionkit.trajectory.ValidationLimits;
import motionkit.trajectory.ValidationReport;
import MotionKitNative;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.StopMode;
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
  /** Joint and sampled Cartesian checks for the last planned path. */
  public var lastPathValidationReport(default, null):Null<ValidationReport> = null;
  public var lastPathPlanningDiagnostics(default, null):Array<String> = [];
  /** Native trajectory currently submitted to the runtime queue. */
  var activeTrajectory:Null<Trajectory> = null;
  var queuedTrajectories:Array<Trajectory> = [];
  var activeSegments:Array<{timeFromStartNs:Int64, durationNs:Int64,
    coefficients:Array<Array<Float>>}> = [];
  var activeStationary:Bool = false;
  var elapsedSeconds:Float = 0.0;
  var held:Bool = false;
  /** A native lifecycle command must reach the owner before another plan. */
  var nativeRefillDeferred:Bool = false;
  var bufferedTotalSeconds:Float = 0.0;
  var bufferedCompletedSeconds:Float = 0.0;
  var plannedEndPositions:Null<Array<Float>> = null;
  var trajectorySubmitted:Bool = false;
  /** Native segment index used as the boundary of the next plan chunk. */
  var trajectoryNextSegmentIndex:Int = 0;
  var trajectoryChunkEndSeconds:Float = 0.0;
  var trajectoryChunkStartSeconds:Float = 0.0;
  var nextTrajectoryTag:Int64 = Int64.ofInt(1);
  var trajectoryChunkReferences:Map<String, PlanChunkReference> = new Map();
  var trajectoryFinalTag:Int64 = Int64.ofInt(0);
  var trajectoryFinalEndSeconds:Float = 0.0;
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
      afterStop.push(() -> queueAxes(targets, options));
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
    if (mode == StopMode.Normal && isMotionInProgress()) {
      robot.submit(RobotCommand.Abort);
      clearBufferedMotion();
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
    if (activeTrajectory == null) return true;
    return !syncFromRuntime().trajectoryActive;
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
      pathOptions:Null<PathPlanningOptions>, motionOptions:Null<MotionOptions>):Trajectory {
    if (path == null) throw "Cartesian path is required";
    var options = pathOptions == null ? PathPlanningOptions.exactStopMode() : pathOptions;
    var planningPath = path;
    lastPathPlanningDiagnostics = [];
    if (!options.exactStop && options.blendTolerance > 0.0) {
      var blended = CornerBlender.blend(path, options.blendTolerance * 0.8,
        options.maxBlendTurnAngleRadians);
      planningPath = blended.path;
      lastPathPlanningDiagnostics = blended.diagnostics;
    }
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

    validatePathLimits(planningPath, [xAxis, yAxis, zAxis]);
    var maxVelocity = [for (_ in start) 1e8];
    var maxAcceleration = [for (_ in start) 1e8];
    var requested = motionOptions == null ? new MotionOptions() : motionOptions;
    for (direct in [xAxis, yAxis, zAxis]) {
      var unit = [for (_ in start) 0.0];
      direct.writeLogicalDelta(unit, 1.0);
      for (joint in direct.jointIndices) {
        var scale = Math.abs(unit[joint]);
        maxVelocity[joint] = direct.maxVelocity * scale;
        maxAcceleration[joint] = direct.maxAcceleration * scale;
        if (requested.maxAcceleration > 0.0)
          maxAcceleration[joint] = Math.min(maxAcceleration[joint],
            requested.maxAcceleration * scale);
      }
    }
    var segments:Array<{timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>}> = [];
    var junctionSpeeds = [for (_ in 0...(planningPath.primitives.length + 1)) 0.0];
    if (!options.exactStop) for (index in 1...planningPath.primitives.length) {
      var before = planningPath.primitives[index - 1];
      var after = planningPath.primitives[index];
      var beforeTangent = before.tangentAt(before.length());
      var afterTangent = after.tangentAt(0.0);
      var tangentDot = 0.0;
      for (coordinate in 0...3)
        tangentDot += beforeTangent[coordinate] * afterTangent[coordinate];
      if (tangentDot < 0.99999) continue;
      var speed = requested.maxVelocity > 0.0 ? requested.maxVelocity : 1e8;
      var pathAcceleration = 1e8;
      for (side in [before, after]) {
        for (sample in 0...33) {
          var distance = side.length() * sample / 32.0;
          var tangent = side.tangentAt(distance);
          var curvature = side.curvatureAt(distance);
          var prime = [for (_ in start) 0.0];
          var second = [for (_ in start) 0.0];
          for (entry in [{axis: xAxis, prime: tangent[0], second: -tangent[1] * curvature},
              {axis: yAxis, prime: tangent[1], second: tangent[0] * curvature},
              {axis: zAxis, prime: tangent[2], second: 0.0}]) {
            entry.axis.writeLogicalDelta(prime, entry.prime);
            entry.axis.writeLogicalDelta(second, entry.second);
          }
          for (joint in 0...start.length) {
            if (Math.abs(prime[joint]) > 1e-12) {
              speed = Math.min(speed, maxVelocity[joint] / Math.abs(prime[joint]));
              pathAcceleration = Math.min(pathAcceleration,
                maxAcceleration[joint] / Math.abs(prime[joint]));
            }
            if (Math.abs(second[joint]) > 1e-12)
              speed = Math.min(speed,
                Math.sqrt((Std.isOfType(side, QuinticBlend) ? 0.5 : 1.0) *
                  maxAcceleration[joint] / Math.abs(second[joint])));
          }
        }
      }
      speed = Math.min(speed * 0.9,
        0.8 * Math.sqrt(2.0 * pathAcceleration *
          Math.min(before.length(), after.length())));
      junctionSpeeds[index] = speed;
    }
    var offset = Int64.ofInt(0);
    var previousEnd:Null<PathPoint> = null;
    var worstTaskDeviation = 0.0;
    var worstTaskTime = 0.0;
    for (primitiveIndex in 0...planningPath.primitives.length) {
      var primitive = planningPath.primitives[primitiveIndex];
      var length = primitive.length();
      var primitiveStart = primitive.pointAt(0.0);
      if (previousEnd != null && previousEnd.distanceTo(primitiveStart) > 1e-8)
        throw "Path primitives must form a connected path";
      previousEnd = primitive.pointAt(length);
      if (length <= 1e-12) continue;
      var count = 1;
      if (Std.isOfType(primitive, ArcSegment)) {
        var arc:ArcSegment = cast primitive;
        count = Std.int(Math.ceil(Math.abs(arc.sweepAngle) * 16.0));
      }
      if (Std.isOfType(primitive, QuinticBlend)) count = 32;
      var distances:Array<Float> = [];
      var positions:Array<Array<Float>> = [];
      var first:Array<Array<Float>> = [];
      var second:Array<Array<Float>> = [];
      for (index in 0...(count + 1)) {
        var distance = length * index / count;
        var point = primitive.pointAt(distance);
        var tangent = primitive.tangentAt(distance);
        var curvature = primitive.curvatureAt(distance);
        var q = start.copy();
        var qPrime = [for (_ in start) 0.0];
        var qDoublePrime = [for (_ in start) 0.0];
        for (entry in [{axis: xAxis, position: point.x, prime: tangent[0],
            second: -tangent[1] * curvature},
            {axis: yAxis, position: point.y, prime: tangent[1],
              second: tangent[0] * curvature},
            {axis: zAxis, position: point.z, prime: tangent[2], second: 0.0}]) {
          entry.axis.writeLogicalPosition(q, entry.position);
          entry.axis.writeLogicalDelta(qPrime, entry.prime);
          entry.axis.writeLogicalDelta(qDoublePrime, entry.second);
        }
        distances.push(distance);
        positions.push(q);
        first.push(qPrime);
        second.push(qDoublePrime);
      }
      var jointPath = new JointPathSamples(distances, positions, first, second);
      var speedCaps = requested.maxVelocity > 0.0
        ? [for (_ in 0...count) requested.maxVelocity] : [];
      // Leave room for nanosecond stage rounding and Hermite coefficient
      // roundoff before the runtime validates exact polynomial extrema.
      var limits = new PathTimingLimits(
        [for (value in maxVelocity) value * 0.999],
        [for (value in maxAcceleration) value * 0.999], speedCaps,
        junctionSpeeds[primitiveIndex], junctionSpeeds[primitiveIndex + 1]);
      var loweringTolerance = options.exactStop ? 1e-6 :
        Math.min(1e-6, options.blendTolerance * 0.01);
      var timed = new ToppraPathTiming(loweringTolerance).time(jointPath, limits);
      var pieceDuration = timed.trajectory.durationSeconds();
      var sampleCount = Std.int(Math.ceil(pieceDuration / 0.001));
      for (sampleIndex in 0...(sampleCount + 1)) {
        var localTime = pieceDuration * sampleIndex / sampleCount;
        var actual = timed.trajectory.evaluate(localTime).positions;
        var tool = new PathPoint(xAxis.logicalPosition(actual),
          yAxis.logicalPosition(actual), zAxis.logicalPosition(actual));
        var deviation = distanceToAuthoredPath(tool, path);
        if (deviation > worstTaskDeviation) {
          worstTaskDeviation = deviation;
          worstTaskTime = Int64.toFloat(offset) * 1e-9 + localTime;
        }
      }
      var pieceSegments = timed.trajectory.segments();
      for (segment in pieceSegments)
        segments.push({timeFromStartNs: Int64.add(offset, segment.timeFromStartNs),
          durationNs: segment.durationNs, coefficients: segment.coefficients});
      var last = pieceSegments[pieceSegments.length - 1];
      offset = Int64.add(offset, Int64.add(last.timeFromStartNs, last.durationNs));
      timed.releaseDistanceMap();
      timed.trajectory.dispose();
    }
    if (segments.length == 0)
      return Trajectory.fromPositionSamples([0.0, fixedTimestepSeconds], [start, start]);
    var finalPoint = previousEnd;
    if (finalPoint == null) throw "Cartesian path has no endpoint";
    var finalPosition = start.copy();
    xAxis.writeLogicalPosition(finalPosition, finalPoint.x);
    yAxis.writeLogicalPosition(finalPosition, finalPoint.y);
    zAxis.writeLogicalPosition(finalPosition, finalPoint.z);
    segments.push({timeFromStartNs: offset, durationNs: Int64.ofInt(1),
      coefficients: [for (position in finalPosition) [position]]});
    var result = Trajectory.fromSegments(segments);
    var validation = new ValidationLimits(start.length, modelRevision, calibrationRevision);
    validation.continuity(0, 1e-9);
    for (joint in 0...start.length) {
      validation.velocity(joint, maxVelocity[joint]);
      validation.acceleration(joint, maxAcceleration[joint]);
    }
    for (direct in [xAxis, yAxis, zAxis]) {
      var lower = start.copy();
      var upper = start.copy();
      direct.writeLogicalPosition(lower, direct.lowerLimit);
      direct.writeLogicalPosition(upper, direct.upperLimit);
      for (joint in direct.jointIndices)
        validation.position(joint, Math.min(lower[joint], upper[joint]),
          Math.max(lower[joint], upper[joint]));
    }
    var report = result.validate(validation);
    var tolerance = options.exactStop || options.blendTolerance == 0.0
      ? 1e-5 : options.blendTolerance;
    report.setTaskSpace(worstTaskDeviation <= tolerance
      ? MotionKitNativeConstants.MK_CHECK_PASSED
      : MotionKitNativeConstants.MK_CHECK_FAILED,
      worstTaskDeviation, worstTaskTime, tolerance, Int64.ofInt(1000000));
    lastPathValidationReport = report;
    if (report.hasFailure()) {
      for (index in 0...report.checks.length) {
        var check = report.checks[index];
        if (check.status == MotionKitNativeConstants.MK_CHECK_FAILED)
          throw 'Timed Cartesian path check $index failed: ${check.value} > ${check.limit} at ${check.timeSeconds}';
      }
    }
    return result;
  }

  function distanceToAuthoredPath(point:PathPoint, path:GeometricPath):Float {
    var closest = Math.POSITIVE_INFINITY;
    for (primitive in path.primitives) {
      if (Std.isOfType(primitive, LineSegment)) {
        var line:LineSegment = cast primitive;
        var length = line.length();
        if (length <= 0.0) {
          closest = Math.min(closest, point.distanceTo(line.start));
          continue;
        }
        var direction = line.tangentAt(0.0);
        var projection = (point.x - line.start.x) * direction[0] +
          (point.y - line.start.y) * direction[1] +
          (point.z - line.start.z) * direction[2];
        closest = Math.min(closest, point.distanceTo(
          line.pointAt(Math.max(0.0, Math.min(length, projection)))));
      } else if (Std.isOfType(primitive, ArcSegment)) {
        var arc:ArcSegment = cast primitive;
        var angle = Math.atan2(point.y - arc.center.y, point.x - arc.center.x);
        var baseShift = Math.round((arc.startAngle - angle) / (2.0 * Math.PI));
        for (shift in -2...3) {
          var candidate = angle + 2.0 * Math.PI * (baseShift + shift);
          var fraction = arc.sweepAngle == 0.0 ? 0.0 :
            (candidate - arc.startAngle) / arc.sweepAngle;
          var distance = arc.length() * Math.max(0.0, Math.min(1.0, fraction));
          closest = Math.min(closest, point.distanceTo(arc.pointAt(distance)));
        }
      } else {
        for (sample in 0...129)
          closest = Math.min(closest,
            point.distanceTo(primitive.pointAt(primitive.length() * sample / 128.0)));
      }
    }
    return closest;
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

  function trySmoothReplacement(executing:Trajectory, jogAxis:Null<String>,
      planFromState:TrajectoryState -> Trajectory):Null<Trajectory> {
    var smooth = executing;
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

  function submitSmoothReplacement(planned:Trajectory, state:TrajectoryState,
      observation:RobotSnapshot, anchorNs:Int64, jogAxis:Null<String>):Trajectory {
    var replacement = planned;
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
    bufferedTotalSeconds = planned.durationSeconds();
    bufferedCompletedSeconds = 0.0;
    plannedEndPositions = trajectoryEnd(planned);
    trajectorySubmitted = true;
    trajectoryNextSegmentIndex = planned.segments().length;
    trajectoryChunkStartSeconds = 0.0;
    trajectoryChunkEndSeconds = planned.durationSeconds();
    trajectoryFinalTag = tag;
    trajectoryFinalEndSeconds = planned.durationSeconds();
    trajectoryChunkReferences.set(Int64.toStr(tag),
      new PlanChunkReference(planned, 0.0));
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
    else if (!trajectorySubmitted || shouldRefillTrajectory(dt)) fillNativeWindow();
    return true;
  }

  public function stop(?mode:StopMode = StopMode.Normal):Void {
    abort(mode);
  }

  /** Starts executing `trajectoryValue` as a fresh, un-retimed plan. */
  function setActive(trajectoryValue:Trajectory):Void {
    activeTrajectory = trajectoryValue;
    activeSegments = trajectoryValue.segments();
    activeStationary = stationarySegments(activeSegments);
    activeJogAxis = null;
    resetExecutionState();
  }

  function resetExecutionState():Void {
    elapsedSeconds = 0.0;
    trajectorySubmitted = false;
    resetTrajectoryChunkState();
    trajectoryChunkReferences = new Map();
    trajectoryFinalTag = Int64.ofInt(0);
    trajectoryFinalEndSeconds = 0.0;
  }

  function planAxesFrom(start:Array<Float>, targets:Array<AxisTarget>,
      options:Null<MotionOptions>):Trajectory {
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
    activeSegments = [];
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
    resetExecutionState();
  }

  function submitActiveTrajectoryChunk():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    var nativeSegments = activeSegments;
    if (nativeSegments.length == 0) return;
    var startIndex = trajectoryNextSegmentIndex;
    if (startIndex >= nativeSegments.length) return;
    var startTimeNs = nativeSegments[startIndex].timeFromStartNs;
    var startTime = Int64.toFloat(startTimeNs) * 1e-9;
    var availableSegments = 4096 - robot.snapshot().trajectoryQueueDepth;
    var segmentLimit = Std.int(Math.min(128, availableSegments));
    if (segmentLimit < 1) return;
    var endIndex = Std.int(Math.min(nativeSegments.length, startIndex + segmentLimit));
    var tag = nextTrajectoryTag;
    nextTrajectoryTag = Int64.add(nextTrajectoryTag, Int64.ofInt(1));
    var segments:Array<TrajectorySegment> = [];
    for (index in startIndex...endIndex) {
      var segment = nativeSegments[index];
      segments.push(new TrajectorySegment(
        Int64.sub(segment.timeFromStartNs, startTimeNs),
        segment.durationNs, segment.coefficients));
    }
    var startState = trajectoryValue.evaluate(startTime);
    var startPosition = startState.positions;
    var startVelocity = startState.velocities;
    var startAcceleration = startState.accelerations;
    if (nativeSegments[startIndex].coefficients[0].length == 2) {
      startVelocity = [for (_ in startPosition) 0.0];
      if (startIndex > 0) {
        var before = nativeSegments[startIndex - 1];
        for (joint in 0...startVelocity.length)
          startVelocity[joint] = before.coefficients[joint][1];
      }
      startAcceleration = [for (_ in startPosition) 0.0];
    }
    var accelerationTolerance = [for (_ in startPosition) 0.0];
    if (nativeSegments[startIndex].coefficients[0].length != 2) {
      var previousAcceleration = startIndex == 0
        ? [for (_ in startPosition) 0.0]
        : trajectoryValue.evaluate(Math.max(0.0, startTime - 1e-9)).accelerations;
      for (joint in 0...startPosition.length)
        accelerationTolerance[joint] =
          Math.abs(startAcceleration[joint] - previousAcceleration[joint]) + 1e-5;
    }
    try robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
      modelRevision, calibrationRevision, 1, startPosition, startVelocity,
      startAcceleration, segments, null, null, null, null, accelerationTolerance,
      endIndex >= nativeSegments.length))) catch (error:Dynamic)
      throw 'plan chunk [$startIndex,$endIndex] of ${nativeSegments.length}: $error';
    trajectorySubmitted = true;
    trajectoryNextSegmentIndex = endIndex;
    trajectoryChunkStartSeconds = startTime;
    var finalSegment = nativeSegments[endIndex - 1];
    trajectoryChunkEndSeconds = Int64.toFloat(Int64.add(finalSegment.timeFromStartNs,
      finalSegment.durationNs)) * 1e-9;
    trajectoryChunkReferences.set(Int64.toStr(tag),
      new PlanChunkReference(trajectoryValue, startTime));
    if (endIndex >= nativeSegments.length) {
      trajectoryFinalTag = tag;
      trajectoryFinalEndSeconds = trajectoryChunkEndSeconds;
    }
  }

  /** Keep a fixed execution window queued so native HOLD never needs a host lead estimate. */
  function fillNativeWindow():Void {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return;
    while (trajectoryNextSegmentIndex < activeSegments.length &&
        trajectoryChunkEndSeconds - elapsedSeconds < 2.0) {
      var before = trajectoryNextSegmentIndex;
      submitActiveTrajectoryChunk();
      if (trajectoryNextSegmentIndex == before) break;
    }
  }

  function syncFromRuntime():RobotSnapshot {
    var observation = robot.snapshot();
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null) return observation;
    var reference = trajectoryChunkReferences.get(Int64.toStr(observation.trajectoryTag));
    if (reference != null && reference.trajectory == trajectoryValue) {
      var runtimeSeconds = Int64.toFloat(observation.trajectoryTagTimeNs) /
        1000000000.0;
      if (Math.isFinite(runtimeSeconds))
        elapsedSeconds = Math.min(trajectoryValue.durationSeconds(),
          Math.max(0.0, reference.startSeconds + runtimeSeconds));
    }
    return observation;
  }

  function trajectoryFinishedInRuntime(observation:RobotSnapshot):Bool {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || !trajectorySubmitted ||
        Int64.compare(trajectoryFinalTag, Int64.ofInt(0)) == 0)
      return false;
    if (trajectoryFinalEndSeconds < trajectoryValue.durationSeconds() - 1e-9)
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
    if (activeTrajectory == null) return;
    bufferedCompletedSeconds += activeTrajectory.durationSeconds();
    activeTrajectory = null;
    activeSegments = [];
    activeStationary = false;
    resetExecutionState();
    activateNextTrajectory();
  }

  function discardNativeTrajectory(value:Trajectory):Void {
    value.dispose();
  }

  function shouldRefillTrajectory(dt:Float):Bool {
    var trajectoryValue = activeTrajectory;
    if (trajectoryValue == null || trajectoryNextSegmentIndex >= activeSegments.length)
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
    trajectoryNextSegmentIndex = 0;
    trajectoryChunkEndSeconds = 0.0;
    trajectoryChunkStartSeconds = 0.0;
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

private class PlanChunkReference {
  public final trajectory:Trajectory;
  public final startSeconds:Float;

  public function new(trajectory:Trajectory, startSeconds:Float) {
    this.trajectory = trajectory;
    this.startSeconds = startSeconds;
  }
}

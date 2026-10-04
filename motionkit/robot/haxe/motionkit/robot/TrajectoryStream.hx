package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.TrajectoryState;
import robotkit.runtime.RobotRuntimeError;
import RobotKitRuntime;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.StopMode;
import robotkit.world.ProcessTimedEvent;
import robotkit.world.SegmentArrays;
import robotkit.world.TrajectorySegment;

private typedef StreamSegment = {timeFromStartNs:Int64, durationNs:Int64,
  coefficients:Array<Array<Float>>};

/** The segments a stream submits: their timing, and each one's degree and coefficients. */
interface StreamSegments {
  function count():Int;
  function jointCount():Int;
  function startNs(index:Int):Int64;
  function durationNs(index:Int):Int64;
  function degree(index:Int):Int;
  function coefficient(index:Int, joint:Int, power:Int):Float;
}

/** Segments held as Haxe values, such as a trajectory's. */
class ArrayStreamSegments implements StreamSegments {
  final segments:Array<StreamSegment>;

  public function new(segments:Array<StreamSegment>) this.segments = segments;

  public function count():Int return segments.length;
  public function jointCount():Int return segments.length == 0 ? 0 : segments[0].coefficients.length;
  public function startNs(index:Int):Int64 return segments[index].timeFromStartNs;
  public function durationNs(index:Int):Int64 return segments[index].durationNs;
  public function degree(index:Int):Int return segments[index].coefficients[0].length - 1;
  public function coefficient(index:Int, joint:Int, power:Int):Float
    return segments[index].coefficients[joint][power];
}

/** A plan's segments read in place from native arrays, over the robot's joints. */
class NativeStreamSegments implements StreamSegments {
  public final arrays:SegmentArrays;

  public function new(arrays:SegmentArrays) this.arrays = arrays;

  public function count():Int return arrays.count();
  public function jointCount():Int return arrays.robotJointCount();
  public function startNs(index:Int):Int64 return arrays.starts.get(index);
  public function durationNs(index:Int):Int64 return arrays.durations.get(index);
  public function degree(index:Int):Int return arrays.degrees.get(index);
  public function coefficient(index:Int, joint:Int, power:Int):Float
    return arrays.coefficient(index, joint, power);
}

/** Bounded owner-clock stream shared by machine motion and validated programs. */
class TrajectoryStream {
  static var nextProgramTag:Int64 = Int64.ofInt(1000000);

  public final robot:Robot;
  public var elapsedSeconds(default, null):Float = 0.0;
  public var submitted(default, null):Bool = false;
  public var nextSegment(default, null):Int = 0;
  public var chunkStartSeconds(default, null):Float = 0.0;
  public var chunkEndSeconds(default, null):Float = 0.0;
  public var finalTag(default, null):Int64 = Int64.ofInt(0);
  public var finalEndSeconds(default, null):Float = 0.0;
  public var finalDurationNs(default, null):Int64 = Int64.ofInt(0);
  var segments:StreamSegments = new ArrayStreamSegments([]);
  var durationSeconds:Float = 0.0;
  var references:Map<String, Float> = new Map();
  var nextMotionTag:Int64 = Int64.ofInt(1);
  final programTags:Bool;

  public function new(robot:Robot, ?programTags:Bool = false) {
    this.robot = robot;
    this.programTags = programTags;
  }

  public function begin(segments:Array<StreamSegment>, durationSeconds:Float):Void
    beginSegments(new ArrayStreamSegments(segments), durationSeconds);

  public function beginSegments(segments:StreamSegments, durationSeconds:Float):Void {
    this.segments = segments;
    this.durationSeconds = durationSeconds;
    elapsedSeconds = 0.0;
    submitted = false;
    nextSegment = 0;
    chunkStartSeconds = 0.0;
    chunkEndSeconds = 0.0;
    finalTag = Int64.ofInt(0);
    finalEndSeconds = 0.0;
    finalDurationNs = Int64.ofInt(0);
    references = new Map();
  }

  public function clear():Void begin([], 0.0);

  public function markCompleted():Void elapsedSeconds = durationSeconds;

  public function sync():RobotSnapshot {
    var observation = robot.snapshot();
    var start = references.get(Int64.toStr(observation.trajectoryTag));
    if (start != null)
      elapsedSeconds = elapsedFromSnapshot(observation, start, durationSeconds);
    return observation;
  }

  /** Observe and submit through the same fault boundary for axes and arms. */
  public function observe(session:MotionSession, snapshot:RobotSnapshot):RobotSnapshot {
    session.observe(snapshot);
    session.requireReady();
    return snapshot;
  }

  public function submit(session:MotionSession, command:RobotCommand):Void {
    try robot.submit(command) catch (error:Dynamic) {
      session.reject();
      throw error;
    }
    observe(session, robot.snapshot());
  }

  public function stop(session:MotionSession, mode:StopMode):Void {
    try robot.stop(mode) catch (error:Dynamic) {
      session.reject();
      throw error;
    }
    observe(session, robot.snapshot());
  }

  public static function atRest(snapshot:RobotSnapshot):Bool
    return (snapshot.sessionState == RobotKitRuntimeConstants.RK_SESSION_HELD ||
      !snapshot.trajectoryActive) &&
      snapshot.sessionState != RobotKitRuntimeConstants.RK_SESSION_STOPPING;

  public static function elapsedFromSnapshot(observation:RobotSnapshot,
      startSeconds:Float, durationSeconds:Float):Float {
    var runtimeSeconds = Int64.toFloat(observation.trajectoryTagTimeNs) * 1e-9;
    return Math.isFinite(runtimeSeconds) ?
      Math.min(durationSeconds, Math.max(0.0, startSeconds + runtimeSeconds)) : 0.0;
  }

  /** Seconds of motion a stream keeps submitted ahead of the robot. */
  public static inline final BUFFER_SECONDS = 2.0;
  /**
    Seconds of motion one call adds at most. The robot checks every segment it
    takes, so topping the buffer up a little a tick keeps that cost even
    instead of a burst when a plan starts or the buffer runs low.
  **/
  public static inline final CHUNK_SECONDS = 0.1;
  /** Seconds of motion always submitted ahead of the robot, however long its ticks. */
  public static inline final MIN_LEAD_SECONDS = 0.25;

  /**
    Keeps a bounded window of motion submitted ahead of the robot, whatever
    the transport and within the room left in its queue: one submission a
    call, of `CHUNK_SECONDS` or as much as brings the robot's lead to
    `MIN_LEAD_SECONDS`, until `BUFFER_SECONDS` is ahead.
  **/
  public function fill(session:MotionSession,
      build:Int -> Int -> Int64 -> Int64 -> Int64 -> ExecutionPlanSubmission,
      describeFailure:Int -> Int -> Dynamic -> String):Void {
    var total = segments.count();
    var ahead = chunkEndSeconds - elapsedSeconds;
    if (nextSegment >= total || ahead >= BUFFER_SECONDS) return;
    // A submission is bounded only by the room left in the runtime's queue.
    var available = RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_QUEUE_POINTS -
      robot.snapshot().trajectoryQueueDepth;
    var room = Std.int(Math.min(available, total - nextSegment));
    if (room < 1) return;
    var horizonNs = Trajectory.nanoseconds(Math.max(elapsedSeconds, chunkEndSeconds) +
      Math.max(CHUNK_SECONDS, MIN_LEAD_SECONDS - ahead));
    var count = 1;
    while (count < room && Int64.compare(segments.startNs(nextSegment + count), horizonNs) < 0)
      count++;
    var first = nextSegment;
    var last = first + count;
    var startNs = segments.startNs(first);
    var endNs = Int64.add(segments.startNs(last - 1), segments.durationNs(last - 1));
    var tag = programTags ? nextProgramTag : nextMotionTag;
    if (programTags) nextProgramTag = Int64.add(nextProgramTag, Int64.ofInt(1));
    var submission = build(first, last, tag, startNs, endNs);
    try submit(session, RobotCommand.ExecutionPlan(submission)) catch (error:Dynamic)
      throw describeFailure(first, last, error);
    if (!programTags) nextMotionTag = Int64.add(nextMotionTag, Int64.ofInt(1));
    var startSeconds = Int64.toFloat(startNs) * 1e-9;
    references.set(Int64.toStr(tag), startSeconds);
    submitted = true;
    nextSegment = last;
    chunkStartSeconds = startSeconds;
    chunkEndSeconds = Int64.toFloat(endNs) * 1e-9;
    if (last == total) {
      finalTag = tag;
      finalEndSeconds = chunkEndSeconds;
      finalDurationNs = Int64.sub(endNs, startNs);
    }
  }

  public function motionSubmission(trajectory:Trajectory, first:Int, last:Int, tag:Int64,
      startNs:Int64, modelRevision:Int64, calibrationRevision:Int64,
      jerkUnchecked:Bool, jointTolerances:Array<Float>):ExecutionPlanSubmission {
    var startSeconds = Int64.toFloat(startNs) * 1e-9;
    var payload:Array<TrajectorySegment> = [];
    for (index in first...last)
      payload.push(new TrajectorySegment(Int64.sub(segments.startNs(index), startNs),
        segments.durationNs(index), [for (joint in 0...segments.jointCount())
          [for (power in 0...segments.degree(index) + 1) segments.coefficient(index, joint, power)]]));
    var state = trajectory.evaluate(startSeconds);
    var velocity = state.velocities;
    var acceleration = state.accelerations;
    if (segments.degree(first) == 1) {
      velocity = [for (_ in state.positions) 0.0];
      if (first > 0)
        for (joint in 0...velocity.length)
          velocity[joint] = segments.coefficient(first - 1, joint, 1);
      acceleration = [for (_ in state.positions) 0.0];
    }
    var accelerationTolerance = jointTolerances.copy();
    if (!jerkUnchecked && first > 0 && segments.degree(first) != 1) {
      // Ruckig phases meet on a rounded nanosecond boundary. State the clock
      // allowance here; the runtime also bounds it by the compiled acceleration cap.
      var previous = trajectory.evaluate(Math.max(0.0, startSeconds - 1e-9));
      for (joint in 0...state.positions.length)
        accelerationTolerance[joint] = jointTolerances[joint] +
          0.5e-9 * (Math.abs(previous.jerks[joint]) + Math.abs(state.jerks[joint]));
    }
    if (jerkUnchecked && segments.degree(first) != 1) {
      var previousAcceleration = first == 0 ?
        [for (_ in state.positions) 0.0] :
        trajectory.evaluate(Math.max(0.0, startSeconds - 1e-9)).accelerations;
      for (joint in 0...state.positions.length)
        accelerationTolerance[joint] =
          Math.abs(acceleration[joint] - previousAcceleration[joint]) + 10.0 * jointTolerances[joint];
    }
    return new ExecutionPlanSubmission(tag, modelRevision, calibrationRevision, 1,
      state.positions, velocity, acceleration, payload, null, null,
      jointTolerances, jointTolerances,
      accelerationTolerance, last == segments.count(), null, jerkUnchecked, null, trajectory.controlAcceleration);
  }

  /**
    Build a program chunk over the robot's joints from the plan's segment
    arrays, which go to the robot in place, after the caller selects events.
  **/
  public function programSubmission(plan:ExecutionPlan, arrays:SegmentArrays, first:Int,
      last:Int, tag:Int64, startNs:Int64, endNs:Int64,
      selectEvents:Int64 -> Int64 -> Bool -> Array<ProcessTimedEvent>,
      endsAtRest:Bool, jerkUnchecked:Bool):ExecutionPlanSubmission {
    var startSeconds = Int64.toFloat(startNs) * 1e-9;
    var state = plan.evaluate(startSeconds);
    var chunk = arrays.slice(first, last);
    var events = selectEvents(startNs, endNs, last == segments.count());
    var fixedPositions = arrays.heldPositions;
    // Joints coupled to planned ones, such as lead screws, start where their leaders put them.
    var positions = arrays.robotValues(state.positions, fixedPositions, true);
    var zero = [for (_ in fixedPositions) 0.0];
    var startVelocity = arrays.robotValues(first == 0 ? plan.copyStartVelocities() :
      state.velocities, zero, false);
    var startAcceleration = arrays.robotValues(first == 0 ? plan.copyStartAccelerations() :
      state.accelerations, zero, false);
    // Linear chunks promise the preceding chord at a continuation anchor.
    if (chunk.degrees.get(0) == 1) {
      startAcceleration = zero.copy();
      startVelocity = first == 0 ? zero.copy() :
        [for (joint in 0...fixedPositions.length) arrays.coefficient(first - 1, joint, 1)];
    }
    // A coupled joint allows its leader's error, scaled by the ratio: a lead screw's turns are
    // its axis's travel times thousands.
    var tolerance = [for (_ in fixedPositions) 0.02];
    var sourceTolerance = [for (_ in state.positions) 0.02];
    var pTol = arrays.robotTolerances(first == 0 ? plan.copyPositionTolerances() : sourceTolerance, tolerance);
    var vTol = arrays.robotTolerances(first == 0 ? plan.copyVelocityTolerances() : sourceTolerance, tolerance);
    var aTol = arrays.robotTolerances(first == 0 ? plan.copyAccelerationTolerances() : sourceTolerance, tolerance);
    var scale = arrays.robotTolerances([for (_ in state.positions) 1.0], [for (_ in fixedPositions) 1.0]);
    // Retain the authored tolerances; the runtime caps checked-C2 seams by its
    // nanosecond rounding bound in the robot's actual joint units.
    if (!jerkUnchecked && first > 0)
      aTol = arrays.robotTolerances(plan.copyAccelerationTolerances(), tolerance);
    if (chunk.degrees.get(0) > 1) {
      var precedingAcceleration = first == 0 ? zero : arrays.robotValues(
        plan.evaluate(Math.max(0.0, startSeconds - 1e-9)).accelerations, zero, false);
      for (joint in 0...fixedPositions.length) {
        var polynomialAcceleration = 2.0 * chunk.coefficient(0, joint, 2);
        if (jerkUnchecked || first == 0) {
          aTol[joint] = Math.max(aTol[joint],
            Math.abs(polynomialAcceleration - startAcceleration[joint]) + 1e-5 * scale[joint]);
          aTol[joint] = Math.max(aTol[joint],
            Math.abs(polynomialAcceleration - precedingAcceleration[joint]) + 1e-5 * scale[joint]);
        }
        startAcceleration[joint] = polynomialAcceleration;
      }
    }
    return ExecutionPlanSubmission.ofArrays(tag, plan.modelRevision,
      plan.calibrationRevision, 1, positions, startVelocity, startAcceleration,
      chunk, pTol, vTol, aTol, last == segments.count() && endsAtRest, events, jerkUnchecked);
  }

  public function finishedMotion(observation:RobotSnapshot):Bool {
    if (!submitted || Int64.compare(finalTag, Int64.ofInt(0)) == 0 ||
        finalEndSeconds < durationSeconds - 1e-9 ||
        Int64.compare(observation.trajectoryTag, finalTag) != 0)
      return false;
    var finalTime = Int64.toFloat(observation.trajectoryTagTimeNs) * 1e-9;
    return Math.isFinite(finalTime) &&
      finalEndSeconds - chunkStartSeconds <= finalTime + 1e-9 &&
      !observation.trajectoryActive && observation.trajectoryQueueDepth == 0;
  }

  public function finishedProgram(observation:RobotSnapshot):Bool
    return Int64.compare(finalTag, Int64.ofInt(0)) != 0 &&
      Int64.compare(observation.trajectoryTag, finalTag) == 0 &&
      Int64.compare(observation.trajectoryTagTimeNs, finalDurationNs) >= 0 &&
      !observation.trajectoryActive && observation.trajectoryQueueDepth == 0;

  public function shouldRefill(dt:Float, fixedTimestepSeconds:Float):Bool {
    if (nextSegment >= segments.count()) return false;
    var lead = Math.max(BUFFER_SECONDS, Math.max(fixedTimestepSeconds, dt) * 2.0);
    if (chunkEndSeconds - elapsedSeconds <= lead + 1e-9) return true;
    if (elapsedSeconds <= 1e-9) return false;
    var observation = robot.snapshot();
    return !observation.trajectoryActive || observation.trajectoryQueueDepth == 0;
  }

  /** Record a smooth replacement already accepted by the runtime. */
  public function recordReplacement(tag:Int64):Void {
    submitted = true;
    nextSegment = segments.count();
    chunkStartSeconds = 0.0;
    chunkEndSeconds = durationSeconds;
    finalTag = tag;
    finalEndSeconds = durationSeconds;
    finalDurationNs = Trajectory.nanoseconds(durationSeconds);
    references.set(Int64.toStr(tag), 0.0);
  }

  public function submitSmoothReplacement(planned:Trajectory, state:TrajectoryState,
      observation:RobotSnapshot, anchorNs:Int64, modelRevision:Int64,
      calibrationRevision:Int64, jointTolerances:Array<Float>):Int64 {
    var tag = nextMotionTag;
    var payload = [for (segment in planned.segments())
      new TrajectorySegment(segment.timeFromStartNs, segment.durationNs,
        segment.coefficients)];
    robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
      modelRevision, calibrationRevision, 1, state.positions, state.velocities,
      state.accelerations, payload, observation.activePlanId, anchorNs,
      jointTolerances, jointTolerances, jointTolerances, true, null, false, null, planned.controlAcceleration)));
    nextMotionTag = Int64.add(nextMotionTag, Int64.ofInt(1));
    return tag;
  }

  /** Reobserve the committed horizon after a rejected replacement. */
  public function replaceWithRetry(executing:Trajectory,
      observe:Void -> RobotSnapshot, ownerPeriodSeconds:Float, marginPeriods:Int,
      planFromState:TrajectoryState -> Trajectory,
      submit:Trajectory -> TrajectoryState -> RobotSnapshot -> Int64 -> Trajectory
      ):Null<Trajectory> {
    for (_ in 0...2) {
      var observation = observe();
      if (!observation.trajectoryActive ||
          Int64.compare(observation.activePlanId, Int64.ofInt(0)) == 0)
        return null;
      // Tags name streamed chunks. Recover the trajectory origin using the
      // chunk's recorded offset, rather than treating every tag as time zero.
      var chunkStart = references.get(Int64.toStr(observation.trajectoryTag));
      var localNs = Int64.ofInt(0);
      if (chunkStart != null)
        localNs = Int64.add(Trajectory.nanoseconds(chunkStart), observation.trajectoryTagTimeNs);
      else if (Int64.compare(observation.trajectoryTimeNs, Int64.ofInt(0)) != 0)
        return null;
      // Before the first owner cycle the accepted queue has no observed tag
      // yet. Its trajectory clock is still zero, so that origin is known.
      var startNs = Int64.sub(observation.trajectoryTimeNs, localNs);
      var marginNs = Trajectory.nanoseconds(ownerPeriodSeconds * marginPeriods);
      var anchorNs = Int64.add(observation.committedUntilNs, marginNs);
      var localSeconds = Int64.toFloat(Int64.sub(anchorNs, startNs)) * 1e-9;
      if (localSeconds < 0.0 ||
          localSeconds >= executing.durationSeconds() - ownerPeriodSeconds)
        return null;
      var state = executing.evaluate(localSeconds);
      var planned = planFromState(state);
      try {
        return submit(planned, state, observation, anchorNs);
      } catch (error:RobotRuntimeError) {
        planned.dispose();
        if (error.status != RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE)
          throw error;
      } catch (error:Dynamic) {
        planned.dispose();
        throw error;
      }
    }
    return null;
  }
}

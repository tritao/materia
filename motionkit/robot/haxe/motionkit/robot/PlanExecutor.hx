package motionkit.robot;

import haxe.Int64;
import motionkit.event.TimedEvent;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.TrajectoryState;
import robotkit.runtime.RobotRuntimeError;
import RobotKitRuntime;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.ProcessEventValue;
import robotkit.world.ProcessHoldPolicy;
import robotkit.world.ProcessTimedEvent;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.TrajectorySegment;

/** Submits a validated plan in bounded chunks and tracks its owner-clock progress. */
class PlanExecutor {
  static var nextTag:Int64 = Int64.ofInt(1000000);
  public final robot:Robot;
  public var elapsedSeconds(default, null):Float = 0.0;
  public var completed(default, null):Bool = false;
  var plan:Null<ExecutionPlan>;
  var planSegments:Array<{timeFromStartNs:Int64, durationNs:Int64,
    coefficients:Array<Array<Float>>}> = [];
  var planEvents:Array<TimedEvent> = [];
  var nextSegment:Int = 0;
  var finalTag:Int64 = Int64.ofInt(0);
  var finalDurationNs:Int64 = Int64.ofInt(0);
  var chunkEndSeconds:Float = 0.0;
  var references:Map<String, Float> = new Map();
  var endsAtRest:Bool = true;
  var deferredRefill:Bool = false;

  public function new(robot:Robot) {
    if (robot == null || !robot.capabilities().supportsExecutionPlans ||
        !robot.capabilities().supportsTrajectoryQueue)
      throw "PlanExecutor requires execution plan and trajectory queue support";
    this.robot = robot;
  }

  public function start(plan:ExecutionPlan, ?endsAtRest:Bool = true):Void {
    if (plan == null || this.plan != null) throw "PlanExecutor needs one inactive validated plan";
    this.plan = plan;
    planSegments = plan.segments();
    planEvents = plan.events;
    this.endsAtRest = endsAtRest;
    elapsedSeconds = 0.0; completed = false; nextSegment = 0;
    finalTag = Int64.ofInt(0); chunkEndSeconds = 0.0;
    references = new Map(); deferredRefill = false;
    fill();
  }

  public function update():Void {
    var active = plan;
    if (active == null || completed) return;
    var observation = sync();
    if (Int64.compare(finalTag, Int64.ofInt(0)) != 0 &&
        Int64.compare(observation.trajectoryTag, finalTag) == 0 &&
        Int64.compare(observation.trajectoryTagTimeNs, finalDurationNs) >= 0 &&
        !observation.trajectoryActive && observation.trajectoryQueueDepth == 0) {
      elapsedSeconds = active.durationSeconds;
      completed = true;
      plan = null;
      return;
    }
    if (deferredRefill) deferredRefill = false; else fill();
  }

  public function hold():Void robot.submit(RobotCommand.Hold);
  public function resume():Void {
    robot.submit(RobotCommand.Resume);
    deferredRefill = true;
  }
  public function abort():Void {
    robot.submit(RobotCommand.Abort);
    plan = null;
    completed = false;
  }

  public function sync():RobotSnapshot {
    var observation = robot.snapshot();
    var active = plan;
    if (active != null) {
      var start = references.get(Int64.toStr(observation.trajectoryTag));
      if (start != null)
        elapsedSeconds = Math.min(active.durationSeconds, Math.max(0.0,
          start + Int64.toFloat(observation.trajectoryTagTimeNs) * 1e-9));
    }
    return observation;
  }

  /** Shared owner-clock conversion used by buffered motion and programs. */
  public static function elapsedFromSnapshot(observation:RobotSnapshot,
      startSeconds:Float, durationSeconds:Float):Float {
    var runtimeSeconds = Int64.toFloat(observation.trajectoryTagTimeNs) * 1e-9;
    return Math.isFinite(runtimeSeconds) ?
      Math.min(durationSeconds, Math.max(0.0, startSeconds + runtimeSeconds)) : 0.0;
  }

  /** Bounded trajectory submission used by MotionSystem's streaming window. */
  public static function submitTrajectoryChunk(robot:Robot, trajectory:Trajectory,
      nativeSegments:Array<{timeFromStartNs:Int64, durationNs:Int64,
        coefficients:Array<Array<Float>>}>, first:Int, tag:Int64,
      modelRevision:Int64, calibrationRevision:Int64):Null<{last:Int,
        startSeconds:Float, endSeconds:Float}> {
    var available = 4096 - robot.snapshot().trajectoryQueueDepth;
    var count = Std.int(Math.min(128, Math.min(available, nativeSegments.length - first)));
    if (count < 1) return null;
    var last = first + count;
    var startNs = nativeSegments[first].timeFromStartNs;
    var startSeconds = Int64.toFloat(startNs) * 1e-9;
    var segments:Array<TrajectorySegment> = [];
    for (index in first...last) {
      var segment = nativeSegments[index];
      segments.push(new TrajectorySegment(Int64.sub(segment.timeFromStartNs, startNs),
        segment.durationNs, segment.coefficients));
    }
    var state = trajectory.evaluate(startSeconds);
    var velocity = state.velocities;
    var acceleration = state.accelerations;
    if (nativeSegments[first].coefficients[0].length == 2) {
      velocity = [for (_ in state.positions) 0.0];
      if (first > 0) {
        var previous = nativeSegments[first - 1];
        for (joint in 0...velocity.length)
          velocity[joint] = previous.coefficients[joint][1];
      }
      acceleration = [for (_ in state.positions) 0.0];
    }
    var accelerationTolerance = [for (_ in state.positions) 0.0];
    if (nativeSegments[first].coefficients[0].length != 2) {
      var previousAcceleration = first == 0 ?
        [for (_ in state.positions) 0.0] :
        trajectory.evaluate(Math.max(0.0, startSeconds - 1e-9)).accelerations;
      for (joint in 0...state.positions.length)
        accelerationTolerance[joint] =
          Math.abs(acceleration[joint] - previousAcceleration[joint]) + 1e-5;
    }
    try robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
      modelRevision, calibrationRevision, 1, state.positions, velocity,
      acceleration, segments, null, null, null, null, accelerationTolerance,
      last == nativeSegments.length))) catch (error:Dynamic)
      throw 'plan chunk [$first,$last] of ${nativeSegments.length}: $error';
    var end = nativeSegments[last - 1];
    return {last:last, startSeconds:startSeconds,
      endSeconds:Int64.toFloat(Int64.add(end.timeFromStartNs, end.durationNs)) * 1e-9};
  }

  /** Reobserve the committed horizon after a rejected replacement. */
  public static function replaceWithRetry(executing:Trajectory,
      observe:Void -> RobotSnapshot, ownerPeriodSeconds:Float, marginPeriods:Int,
      planFromState:TrajectoryState -> Trajectory,
      submit:Trajectory -> TrajectoryState -> RobotSnapshot -> Int64 -> Trajectory
      ):Null<Trajectory> {
    for (_ in 0...2) {
      var observation = observe();
      if (!observation.trajectoryActive ||
          Int64.compare(observation.activePlanId, Int64.ofInt(0)) == 0)
        return null;
      var startNs = Int64.sub(observation.trajectoryTimeNs,
        observation.trajectoryTagTimeNs);
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

  function fill():Void {
    var active = plan;
    if (active == null) return;
    var stagedSegments = 0;
    while (nextSegment < planSegments.length &&
        chunkEndSeconds - elapsedSeconds < 2.0) {
      var available = 4096 - robot.snapshot().trajectoryQueueDepth - stagedSegments;
      var count = Std.int(Math.min(128, Math.min(available,
        planSegments.length - nextSegment)));
      if (count < 1) return;
      var first = nextSegment;
      var last = first + count;
      var startNs = planSegments[first].timeFromStartNs;
      var startSeconds = Int64.toFloat(startNs) * 1e-9;
      var state = active.evaluate(startSeconds);
      var segments:Array<TrajectorySegment> = [];
      for (index in first...last) {
        var segment = planSegments[index];
        segments.push(new TrajectorySegment(Int64.sub(segment.timeFromStartNs, startNs),
          segment.durationNs, segment.coefficients));
      }
      var events:Array<ProcessTimedEvent> = [];
      var endNs = Int64.add(planSegments[last - 1].timeFromStartNs,
        planSegments[last - 1].durationNs);
      for (event in planEvents)
        if (Int64.compare(event.timeNs, startNs) >= 0 &&
            (Int64.compare(event.timeNs, endNs) < 0 ||
             (last == planSegments.length && Int64.compare(event.timeNs, endNs) <= 0))) {
          var value = switch event.value {
            case Digital(enabled): ProcessEventValue.Digital(enabled);
            case Analog(number): ProcessEventValue.Analog(number);
            case Process(command, argument): ProcessEventValue.Process(command, argument);
          };
          var hold = switch event.holdPolicy {
            case Keep: ProcessHoldPolicy.Keep;
            case SafeWhileHeld: ProcessHoldPolicy.SafeWhileHeld;
            case RestoreOnResume: ProcessHoldPolicy.RestoreOnResume;
          };
          events.push(new ProcessTimedEvent(Int64.sub(event.timeNs, startNs),
            event.channel, value, hold));
        }
      var startVelocity = first == 0 ? active.copyStartVelocities() : state.velocities;
      var startAcceleration = first == 0 ? active.copyStartAccelerations() : state.accelerations;
      var pTol = first == 0 ? active.copyPositionTolerances() :
        [for (_ in state.positions) 0.02];
      var vTol = first == 0 ? active.copyVelocityTolerances() :
        [for (_ in state.positions) 0.02];
      var aTol = first == 0 ? active.copyAccelerationTolerances() :
        [for (_ in state.positions) 0.02];
      var tag = nextTag;
      nextTag = Int64.add(nextTag, Int64.ofInt(1));
      robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(tag,
        active.modelRevision, active.calibrationRevision, 1, state.positions,
        startVelocity, startAcceleration, segments, null, null, pTol, vTol, aTol,
        last == planSegments.length && endsAtRest, events)));
      references.set(Int64.toStr(tag), startSeconds);
      nextSegment = last;
      stagedSegments += count;
      chunkEndSeconds = Int64.toFloat(endNs) * 1e-9;
      if (last == planSegments.length) {
        finalTag = tag;
        finalDurationNs = Int64.sub(endNs, startNs);
      }
    }
  }
}

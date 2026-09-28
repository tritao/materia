package motionkit.robot;

import haxe.Int64;
import motionkit.event.TimedEvent;
import motionkit.trajectory.ExecutionPlan;
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
  public final robot:Robot;
  public final jointIndices:Array<Int>;
  public var elapsedSeconds(get, never):Float;
  public var completed(default, null):Bool = false;
  var plan:Null<ExecutionPlan>;
  var planSegments:Array<{timeFromStartNs:Int64, durationNs:Int64,
    coefficients:Array<Array<Float>>}> = [];
  var planEvents:Array<TimedEvent> = [];
  var fixedPositions:Array<Float> = [];
  final stream:TrajectoryStream;
  var endsAtRest:Bool = true;
  var jerkUnchecked:Bool = false;
  var deferredRefill:Bool = false;

  public function new(robot:Robot, ?jointIndices:Array<Int>) {
    if (robot == null || !robot.capabilities().supportsExecutionPlans ||
        !robot.capabilities().supportsTrajectoryQueue)
      throw "PlanExecutor requires execution plan and trajectory queue support";
    this.robot = robot;
    stream = new TrajectoryStream(robot, true);
    this.jointIndices = jointIndices == null ?
      [for (i in 0...robot.description().joints.length) i] : jointIndices.copy();
  }

  public function start(plan:ExecutionPlan, ?endsAtRest:Bool = true):Void {
    if (plan == null || this.plan != null) throw "PlanExecutor needs one inactive validated plan";
    this.plan = plan;
    planSegments = plan.segments();
    planEvents = plan.events;
    fixedPositions = robot.snapshot().positions.toArray();
    if (plan.evaluate(0.0).positions.length != jointIndices.length)
      throw "PlanExecutor plan and robot joint map disagree";
    this.endsAtRest = endsAtRest;
    jerkUnchecked = plan.report.checks[MotionKitNativeConstants.MK_CHECK_JERK].status ==
      MotionKitNativeConstants.MK_CHECK_UNCHECKED;
    completed = false;
    stream.begin(planSegments, plan.durationSeconds);
    deferredRefill = false;
    fill();
  }

  public function update():Void {
    var active = plan;
    if (active == null || completed) return;
    var observation = sync();
    if (stream.finishedProgram(observation)) {
      stream.markCompleted();
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
    stream.clear();
    completed = false;
  }

  function get_elapsedSeconds():Float return stream.elapsedSeconds;

  public function sync():RobotSnapshot
    return plan == null ? robot.snapshot() : stream.sync();

  function fill():Void {
    if (plan == null) return;
    stream.fill(fixedPositions.length, true, buildChunk,
      (first, last, error) -> {
        var snapshot = robot.snapshot();
        return 'plan chunk [$first,$last] of ${planSegments.length}, '
          + 'events=$lastChunkEvents, jerkUnchecked=$jerkUnchecked, safety=${snapshot.safety}, '
          + 'active=${snapshot.trajectoryActive}, queue=${snapshot.trajectoryQueueDepth}: $error';
      });
  }

  var lastChunkEvents:Int = 0;

  function buildChunk(first:Int, last:Int, tag:Int64, startNs:Int64,
      endNs:Int64):ExecutionPlanSubmission {
    var active = plan;
    if (active == null) throw "PlanExecutor needs an active plan";
    return stream.programSubmission(active, first, last, tag, startNs, endNs,
      fixedPositions, expanded, expandSegments, chunkEvents, endsAtRest,
      jerkUnchecked);
  }

  function expandSegments(first:Int, last:Int, startNs:Int64):Array<TrajectorySegment> {
    var payload:Array<TrajectorySegment> = [];
    for (index in first...last) {
      var segment = planSegments[index];
      var degree = segment.coefficients[0].length;
      var coefficients = [for (joint in 0...fixedPositions.length)
        [for (power in 0...degree) power == 0 ? fixedPositions[joint] : 0.0]];
      for (joint in 0...jointIndices.length)
        coefficients[jointIndices[joint]] = segment.coefficients[joint].copy();
      payload.push(new TrajectorySegment(Int64.sub(segment.timeFromStartNs, startNs),
        segment.durationNs, coefficients));
    }
    return payload;
  }

  function chunkEvents(startNs:Int64, endNs:Int64,
      finalChunk:Bool):Array<ProcessTimedEvent> {
    var events:Array<ProcessTimedEvent> = [];
    for (event in planEvents)
      if (Int64.compare(event.timeNs, startNs) >= 0 &&
          (Int64.compare(event.timeNs, endNs) < 0 ||
           (finalChunk && Int64.compare(event.timeNs, endNs) <= 0))) {
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
    lastChunkEvents = events.length;
    return events;
  }

  function expanded(values:Array<Float>, baseline:Array<Float>):Array<Float> {
    var result = baseline.copy();
    for (joint in 0...jointIndices.length)
      result[jointIndices[joint]] = values[joint];
    return result;
  }
}

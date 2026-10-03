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
import robotkit.world.SegmentArrays;

/** Submits a validated plan in bounded chunks and tracks its owner-clock progress. */
class PlanExecutor {
  public final robot:Robot;
  public final jointIndices:Array<Int>;
  public var elapsedSeconds(get, never):Float;
  public var completed(default, null):Bool = false;
  public final session:MotionSession;
  final ownsSession:Bool;
  var plan:Null<ExecutionPlan>;
  /** The active plan's segments, read in place over the robot's joints. */
  var planArrays:Null<SegmentArrays> = null;
  var planEvents:Array<TimedEvent> = [];
  var fixedPositions:Array<Float> = [];
  final stream:TrajectoryStream;
  var endsAtRest:Bool = true;
  var jerkUnchecked:Bool = false;
  var deferredRefill:Bool = false;

  public function new(robot:Robot, ?jointIndices:Array<Int>,
      ?session:MotionSession) {
    if (robot == null || !robot.capabilities().supportsExecutionPlans ||
        !robot.capabilities().supportsTrajectoryQueue)
      throw "PlanExecutor requires execution plan and trajectory queue support";
    this.robot = robot;
    ownsSession = session == null;
    this.session = session == null ? new MotionSession() : session;
    stream = new TrajectoryStream(robot, true);
    this.jointIndices = jointIndices == null ?
      [for (i in 0...robot.description().joints.length) i] : jointIndices.copy();
  }

  public function start(plan:ExecutionPlan, ?endsAtRest:Bool = true):Void {
    session.requireReady();
    if (session.isStopping() || session.isHolding())
      throw "PlanExecutor must reach rest and resume before starting a plan";
    if (plan == null || this.plan != null) throw "PlanExecutor needs one inactive validated plan";
    if (plan.evaluate(0.0).positions.length != jointIndices.length)
      throw "PlanExecutor plan and robot joint map disagree";
    planEvents = plan.events;
    fixedPositions = robot.snapshot().positions.toArray();
    if (ExecutionPlan.COEFFICIENT_STRIDE != SegmentArrays.STRIDE)
      throw "MotionKit and RobotKit disagree on the segment coefficient layout";
    var arrays = new SegmentArrays(plan.segmentStarts(), plan.segmentDurations(),
      plan.segmentDegrees(), plan.segmentCoefficients(), jointIndices, fixedPositions,
      robot.description().couplings);
    planArrays = arrays;
    this.plan = plan;
    this.endsAtRest = endsAtRest;
    jerkUnchecked = plan.report.checks[MotionKitNativeConstants.MK_CHECK_JERK].status ==
      MotionKitNativeConstants.MK_CHECK_UNCHECKED;
    completed = false;
    stream.beginSegments(new TrajectoryStream.NativeStreamSegments(arrays), plan.durationSeconds);
    deferredRefill = false;
    if (session.state == Idle) session.begin();
    fill();
  }

  public function update():Void {
    var observation = sync();
    stream.observe(session, observation);
    if (session.isStopping()) {
      if (TrajectoryStream.atRest(observation)) {
        session.rest();
        clear();
      }
      return;
    }
    if (session.state == Holding) {
      if (TrajectoryStream.atRest(observation)) {
        session.rest();
        if (session.resumePending) {
          resume();
          // The owner tick after this update applies Resume before the next
          // update can refill, so no second defer is needed.
          deferredRefill = false;
        }
      }
      return;
    }
    if (session.state == Held) return;
    var active = plan;
    if (active == null) return;
    if (stream.finishedProgram(observation)) {
      stream.markCompleted();
      completed = true;
      plan = null;
      planArrays = null;
      if (ownsSession) session.completed();
      return;
    }
    if (deferredRefill) deferredRefill = false; else fill();
  }

  public function hold():Void {
    if (session.hold() && plan != null) stream.submit(session, RobotCommand.Hold);
  }
  public function resume():Void {
    if (session.resume() && plan != null) {
      stream.submit(session, RobotCommand.Resume);
      deferredRefill = true;
    }
  }
  public function abort():Void {
    session.requireReady();
    if (session.isStopping()) return;
    if (session.state == Idle) { clear(); return; }
    stream.submit(session, RobotCommand.Abort);
    session.stop(Discard, []);
  }

  public function reset():Void {
    if (!session.isFaulted()) return;
    robot.resetSafety();
    var snapshot = robot.snapshot();
    if (snapshot.faultCode != 0) {
      session.observe(snapshot);
      throw 'Motion runtime fault ${snapshot.faultCode} remains after reset';
    }
    clear();
    session.reset();
  }

  public function clear():Void {
    plan = null;
    planArrays = null;
    stream.clear();
    completed = false;
    deferredRefill = false;
  }

  function get_elapsedSeconds():Float return stream.elapsedSeconds;

  public function sync():RobotSnapshot
    return plan == null ? robot.snapshot() : stream.sync();

  function fill():Void {
    if (plan == null) return;
    stream.fill(session, buildChunk,
      (first, last, error) -> {
        var snapshot = robot.snapshot();
        return 'plan chunk [$first,$last] of ${planArrays == null ? 0 : planArrays.count()}, '
          + 'events=$lastChunkEvents, jerkUnchecked=$jerkUnchecked, safety=${snapshot.safety}, '
          + 'active=${snapshot.trajectoryActive}, queue=${snapshot.trajectoryQueueDepth}: $error';
      });
  }

  var lastChunkEvents:Int = 0;

  function buildChunk(first:Int, last:Int, tag:Int64, startNs:Int64,
      endNs:Int64):ExecutionPlanSubmission {
    var active = plan, arrays = planArrays;
    if (active == null || arrays == null) throw "PlanExecutor needs an active plan";
    return stream.programSubmission(active, arrays, first, last, tag, startNs, endNs,
      chunkEvents, endsAtRest, jerkUnchecked);
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

}

package processkit;

import motionkit.event.EventValue;
import motionkit.event.HoldPolicy;
import motionkit.event.PathEvent;
import motionkit.path.PosePath;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.MotionSession;
import motionkit.robot.SessionState;
import robotkit.execution.FiredProcessEvent;

/** Plans process spans and logs interruption and recovery transitions. */
class ProcessRun {
  public final recipe:ProcessRecipe;
  public final path:PosePath;
  public final device:ProcessDevice;
  public final channel:String;
  public final motionSession:MotionSession;
  public final transitions:Array<ProcessTransition> = [];
  public var state(default, null):ProcessRunState;
  public var currentFeed(default, null):Float;
  public var lastProgramStart(default, null):Float = 0.0;
  public var interruptedAt(default, null):Float = 0.0;
  /** Index, in the program `takeProgram` last returned, of the operation that follows the path. */
  public var followOp(default, null):Int = 0;
  /** Why the run failed to begin, when it is `Failed`. */
  public var failure(default, null):Null<String> = null;
  var pendingBackoff:Float = 0.0;
  var pausedForFeed:Bool = false;
  var preparing:Float = 0.0;

  public function new(recipe:ProcessRecipe, path:PosePath,
      device:ProcessDevice, channel:String, motionSession:MotionSession) {
    if (recipe == null || path == null || path.length() <= 0.0 ||
        device == null || channel == null || StringTools.trim(channel).length == 0 ||
        motionSession == null)
      throw "Process run needs a recipe, path, device, channel and motion session";
    this.recipe = recipe;
    this.path = path;
    this.device = device;
    this.channel = channel;
    this.motionSession = motionSession;
    currentFeed = recipe.nominalSpeed;
  }

  public function start():Void {
    if (state != null) throw "Process run already started";
    device.prepare();
    transition(ProcessRunState.Preparation, 0.0, "prepare device");
  }

  /**
   * Observe a global authored path distance and advance the state machine. `dtSeconds` is the time since the last call,
   * which the recipe's prepare timeout counts while the device is being prepared.
   */
  public function update(distance:Float, ?dtSeconds:Float):Void {
    requireDistance(distance);
    switch state {
      case Preparation:
        if (dtSeconds != null) preparing += dtSeconds;
        var fault = device.fault();
        if (fault == null && device.ready()) {
          transition(ProcessRunState.Ready, distance, "device ready");
          return;
        }
        var limit = recipe.prepareTimeout;
        if (limit != null && preparing > limit) {
          failure = fault == null ? "the device did not become ready" : 'the device did not become ready: $fault';
          transition(ProcessRunState.Failed, distance, failure);
        }
      case Ready:
        if (device.fault() != null) interrupt(distance, "device fault", recipe.recoveryBackoff);
      case Active:
        if (device.fault() != null) interrupt(distance, "device fault", recipe.recoveryBackoff);
        else if (distance >= path.length() && !disengages()) {
          device.safe();
          transition(ProcessRunState.Completion, path.length(), "path complete");
        }
      case ControlledInterruption:
        if (!pausedForFeed && device.fault() == null && canRecover()) {
          device.prepare();
          transition(ProcessRunState.Recovery, interruptedAt, "ready to reapproach");
        }
      case Recovery:
        if (device.fault() != null) interrupt(distance, "device fault", recipe.recoveryBackoff);
      case Completion | Failed | null:
    }
  }

  /**
   * The caller passes this program to ManipulatorMotion; ProcessRun never submits it. A program may not end on an
   * output change, so when the recipe's engagement exit ends on one the caller gives the `closing` motion that follows it.
   */
  public function takeProgram(?closing:MotionOp):MotionProgram {
    var startDistance=availableStart();
    var continuation = ProcessPathSlice.from(path, startDistance);
    var events = processEvents(startDistance, currentFeed);
    var approach = recipe.approachSpeed;
    var engagement = recipe.engagement;
    var ops:Array<MotionOp> = [
      MotionOp.MoveL(continuation.waypointAt(0.0).pose, path.frameId,
        approach == null ? currentFeed : approach, Blend.ExactStop, recipe.orientationPolicy)
    ];
    if (engagement != null) {
      var entry = state == ProcessRunState.Recovery ? engagement.recoveryEntry : engagement.entry;
      for (op in entry) ops.push(op);
    }
    followOp = ops.length;
    ops.push(MotionOp.FollowPath(continuation, path.frameId, currentFeed, events));
    if (engagement != null) for (op in engagement.exit) ops.push(op);
    if (closing != null) ops.push(closing);
    switch ops[ops.length - 1] {
      case SetOutput(_, _): throw "A process program may not end on an output change: give the closing motion that follows the engagement exit";
      default:
    }
    var program = new MotionProgram(ops);
    activatePrepared(startDistance);
    return program;
  }

  /** Activate a caller's already checked program without rebuilding geometry
   * or events. Its start must match the current initial/recovery boundary. */
  public function activatePrepared(startDistance:Float):Void {
    var expected=availableStart();
    if(!Math.isFinite(startDistance) || Math.abs(startDistance-expected)>1e-12)
      throw "Prepared process program does not match the current recovery start";
    lastProgramStart=expected;
    transition(ProcessRunState.Active,expected,expected>0.0?"resume after backoff":"begin process");
  }

  function availableStart():Float {
    if(state==null || state!=ProcessRunState.Ready && state!=ProcessRunState.Recovery)
      throw "Process program is available only when ready or recovering";
    if(pausedForFeed)throw "Process feed override is paused";
    if(state==ProcessRunState.Recovery && !canRecover())
      throw "Process recovery requires held or idle motion";
    if(!device.ready())throw "Process device is not ready";
    var start=state==ProcessRunState.Recovery?Math.max(0.0,interruptedAt-pendingBackoff):0.0;
    if(start>=path.length())throw "Process restart is at the path end";
    return start;
  }

  /**
   * The run is over because the program it handed out is done, exit included: the device is made safe and the run
   * completes. Needed only with an engagement that has an exit; without one the run completes by itself at the
   * path's end.
   */
  public function finish():Void {
    if (state != ProcessRunState.Active) throw "Only an active process run can finish";
    device.safe();
    transition(ProcessRunState.Completion, path.length(), "program complete");
  }

  /**
   * Interrupts the run now, at path `distance`, for a reason the device did not report as a fault: the caller's own
   * watch on the process, such as an arc that never established. Recovery follows as after a fault.
   */
  public function interruptNow(distance:Float, reason:String):Void {
    requireDistance(distance);
    if (state != ProcessRunState.Active && state != ProcessRunState.Ready && state != ProcessRunState.Recovery)
      throw "Only a running process can be interrupted";
    interrupt(distance, reason, recipe.recoveryBackoff);
  }

  function disengages():Bool {
    var engagement = recipe.engagement;
    return engagement != null && engagement.exit.length > 0;
  }

  function canRecover():Bool return motionSession.state == SessionState.Held ||
    motionSession.state == SessionState.Idle;

  /** Apply runtime-fired process records to the bound simulation device. */
  public function applyRecords(records:Array<FiredProcessEvent>):Void device.apply(records);

  /** Request a feed change at the currently executed path distance. */
  public function requestFeed(speed:Float, distance:Float):Void {
    requireDistance(distance);
    if (!Math.isFinite(speed) || speed <= 0.0) throw "Process feed must be positive";
    switch recipe.feedChangePolicy {
      case Reject:
        if (speed != currentFeed) throw "Process recipe rejects feed overrides";
      case Pause:
        if (speed != recipe.nominalSpeed) {
          pausedForFeed = true;
          if (state == ProcessRunState.Active)
            interrupt(distance, "feed override paused", 0.0);
        } else {
          pausedForFeed = false;
          currentFeed = speed;
        }
      case Adapt:
        recipe.rateForSpeed(speed);
        if (speed != currentFeed && state == ProcessRunState.Active)
          interrupt(distance, "adapt feed", 0.0);
        currentFeed = speed;
    }
  }

  function interrupt(distance:Float, reason:String, backoff:Float):Void {
    interruptedAt = distance;
    // The restart backs up from where the process had got to. An interruption before it got beyond the point this
    // program began at (a re-strike that failed) laid nothing new, so it retries that same point and does not back up again.
    pendingBackoff = distance > lastProgramStart + 1e-9 ? backoff : 0.0;
    device.safe();
    transition(ProcessRunState.ControlledInterruption, distance, reason);
  }

  function processEvents(startDistance:Float, feed:Float):Array<PathEvent> {
    var active = true;
    for (event in path.events) if (event.channel == channel) {
      active = false;
      break;
    }
    for (event in path.events) {
      if (event.distance > startDistance) break;
      if (event.channel != channel) continue;
      active = enabled(event.value);
    }
    var remaining = path.length() - startDistance;
    var events = [new PathEvent(0.0, channel,
      EventValue.Analog(active ? recipe.rateForSpeed(feed) : 0.0),
      active ? recipe.triggerLeadSeconds : 0.0, HoldPolicy.RestoreOnResume)];
    for (event in path.events) {
      if (event.distance <= startDistance || event.distance >= path.length()) continue;
      if (event.channel != channel) continue;
      active = enabled(event.value);
      events.push(new PathEvent(event.distance - startDistance, channel,
        EventValue.Analog(active ? recipe.rateForSpeed(feed) : 0.0),
        active ? recipe.triggerLeadSeconds : 0.0, HoldPolicy.RestoreOnResume));
    }
    if (!disengages()) events.push(new PathEvent(remaining, channel, EventValue.Analog(0.0)));
    return events;
  }

  static function enabled(value:EventValue):Bool return switch value {
    case Digital(on): on;
    case Analog(rate): rate > 0.0;
    case Process(_, amount): amount > 0.0;
  };

  function requireDistance(distance:Float):Void {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > path.length())
      throw "Observed process distance lies outside path";
  }

  function transition(next:ProcessRunState, distance:Float, reason:String):Void {
    state = next;
    transitions.push(new ProcessTransition(next, distance, reason));
  }
}

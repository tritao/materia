package toolpathkit.motion;

import motionkit.event.EventValue;
import motionkit.event.PathEvent;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.SessionState;
import processkit.ProcessPathSlice;

/** Runs a machining program and prepares a safe mid-path continuation. */
class MachiningRun {
  public final recipe:MachiningRecipe;
  public final program:MotionProgram;
  public final motion:ManipulatorMotion;
  public var spindleFaulted(default, null):Bool = false;

  public function new(recipe:MachiningRecipe, program:MotionProgram,
      motion:ManipulatorMotion) {
    if (recipe == null || program == null || motion == null)
      throw "machining run needs a recipe, program and motion controller";
    this.recipe = recipe;
    this.program = program;
    this.motion = motion;
  }

  public function start():Void {
    if (spindleFaulted) throw "reset spindle fault before machining";
    motion.run(program);
  }

  public function feedHold():Void {
    if (!motion.running) throw "feed hold needs active machining";
    motion.hold();
  }

  /** Aborts a held execution and returns the program to submit once idle. */
  public function prepareRestart(opIndex:Int, distance:Float):MotionProgram {
    if (spindleFaulted) throw "cannot restart after spindle fault";
    if (motion.sessionState() != SessionState.Held)
      throw "machining restart needs a completed feed hold";
    if (opIndex < 0 || opIndex >= program.ops.length)
      throw "machining restart operation is outside the program";
    var path:motionkit.path.PosePath = null;
    var frame = "", feed = 0.0;
    var sourceEvents:Array<PathEvent> = [];
    switch program.ops[opIndex] {
      case FollowPath(value, frameId, timing, events):
        path = value; frame = frameId; feed = timing;
        sourceEvents = events;
      case _: throw "machining restart requires a path operation";
    }
    if (!Math.isFinite(distance) || distance < 0.0 || distance >= path.length())
      throw "machining restart distance lies outside the path";
    var start = Math.max(0.0, distance - recipe.restartBackoff);
    var states:Map<String, EventValue> = new Map();
    for (index in 0...(opIndex + 1)) {
      var limit = index == opIndex ? start : Math.POSITIVE_INFINITY;
      switch program.ops[index] {
        case SetOutput(channel, value): states.set(channel, value);
        case FollowPath(_, _, _, events):
          for (event in events)
            if (event.distance <= limit) states.set(event.channel, event.value);
        case _:
      }
    }
    var continuation = ProcessPathSlice.from(path, start);
    var remaining:Array<PathEvent> = [];
    for (event in sourceEvents)
      if (event.distance > start)
        remaining.push(new PathEvent(event.distance - start, event.channel,
          event.value, event.leadSeconds, event.holdPolicy));
    var result:Array<MotionOp> = [];
    for (channel in [ToolpathChannels.SpindleDirection,
        ToolpathChannels.SpindleSpeed, ToolpathChannels.CoolantMist,
        ToolpathChannels.CoolantFlood]) {
      var value = states.get(channel);
      if (value != null) result.push(MotionOp.SetOutput(channel, value));
    }
    var spindle = states.get(ToolpathChannels.SpindleSpeed);
    if (spindle != null && switch spindle {
      case Analog(rpm): rpm > 0.0;
      case _: false;
    })
      result.push(MotionOp.WaitInput(ToolpathChannels.SpindleAtSpeed,
        InputPredicate.Equals(EventValue.Digital(true)), null));
    result.push(MotionOp.MoveL(continuation.waypointAt(0.0).pose,
      frame, recipe.approachFeed, Blend.ExactStop));
    result.push(MotionOp.FollowPath(continuation, frame, feed, remaining));
    for (index in (opIndex + 1)...program.ops.length)
      result.push(program.ops[index]);
    motion.abort();
    return new MotionProgram(result);
  }

  public function resumePrepared(continuation:MotionProgram):Void {
    if (spindleFaulted) throw "cannot resume after spindle fault";
    var snapshot = motion.robot.snapshot();
    if (motion.running || motion.sessionState() != SessionState.Idle ||
        snapshot.trajectoryActive || snapshot.trajectoryQueueDepth > 0 ||
        snapshot.safety != 0)
      throw "machining restart needs a controlled stop";
    motion.run(continuation);
  }

  /** Requests a controlled stop; call safeShutdown after the stop completes. */
  public function spindleFault():Void {
    spindleFaulted = true;
    motion.abort();
  }

  public function safeShutdown():MotionProgram {
    var snapshot = motion.robot.snapshot();
    if (!spindleFaulted || motion.running ||
        motion.sessionState() != SessionState.Idle ||
        snapshot.trajectoryActive || snapshot.trajectoryQueueDepth > 0 ||
        snapshot.safety != 0)
      throw "spindle fault shutdown waits for a controlled stop";
    return new MotionProgram([
      MotionOp.SetOutput(ToolpathChannels.SpindleSpeed,
        EventValue.Analog(0.0)),
      MotionOp.SetOutput(ToolpathChannels.SpindleDirection,
        EventValue.Analog(0.0)),
      MotionOp.SetOutput(ToolpathChannels.CoolantMist,
        EventValue.Digital(false)),
      MotionOp.SetOutput(ToolpathChannels.CoolantFlood,
        EventValue.Digital(false))
    ]);
  }
}

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
import motionkit.kinematics.Pose3;

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
  public function prepareRestart(opIndex:Int, distance:Float):MachiningContinuation {
    if (spindleFaulted) throw "cannot restart after spindle fault";
    if (motion.sessionState() != SessionState.Held)
      throw "machining restart needs a completed feed hold";
    var continuation = continuationFrom(opIndex, distance);
    motion.abort();
    return continuation;
  }

  /**
    The program that resumes machining at `distance` along path op
    `opIndex`, from where the machine stands: it loads the tool the program
    had loaded there, restores the spindle and coolant, approaches (over
    the recipe's clearance, when it has one) and carries on.
  **/
  public function continuationFrom(opIndex:Int, distance:Float):MachiningContinuation {
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
    var toolChange:Null<MotionOp> = null;
    for (index in 0...(opIndex + 1)) {
      var limit = index == opIndex ? start : Math.POSITIVE_INFINITY;
      switch program.ops[index] {
        case SetOutput(channel, value): states.set(channel, value);
        case WaitInput(channel, _, _) if (StringTools.startsWith(channel, ToolpathChannels.ToolChangePrefix)):
          toolChange = program.ops[index];
        case FollowPath(_, _, _, events):
          for (event in events)
            if (event.distance <= limit) states.set(event.channel, event.value);
        case _:
      }
    }
    var continuation = ProcessPathSlice.from(path, start);
    var remaining:Array<PathEvent> = [];
    // The slice sums its primitives' lengths, which can fall a rounding error short
    // of the source's; an event at the source's end stays at the slice's.
    for (event in sourceEvents)
      if (event.distance > start)
        remaining.push(new PathEvent(Math.min(continuation.length(), event.distance - start),
          event.channel, event.value, event.leadSeconds, event.holdPolicy));
    var result:Array<MotionOp> = [];
    var clearance = recipe.clearanceZ;
    var target = continuation.waypointAt(0.0).pose;
    if (clearance != null) {
      var here = motion.compiler.solver.forward(currentJoints());
      var up = new Pose3(here.x, here.y, clearance);
      var across = new Pose3(target.x, target.y, clearance);
      if (Math.abs(here.z - clearance) > 1e-9)
        result.push(MotionOp.MoveL(up, frame, recipe.approachFeed, Blend.ExactStop));
      if (up.x != across.x || up.y != across.y)
        result.push(MotionOp.MoveL(across, frame, recipe.approachFeed, Blend.ExactStop));
    }
    // The tool goes in before the spindle starts, as the program did.
    if (toolChange != null) result.push(toolChange);
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
    result.push(MotionOp.MoveL(target, frame, recipe.approachFeed, Blend.ExactStop));
    var resumeOp = result.length;
    result.push(MotionOp.FollowPath(continuation, frame, feed, remaining));
    for (index in (opIndex + 1)...program.ops.length)
      result.push(program.ops[index]);
    return new MachiningContinuation(new MotionProgram(result), opIndex, start, resumeOp);
  }

  /** Where the machine's joints stand: at rest, when a restart is planned. */
  function currentJoints():Array<Float> {
    var positions = motion.robot.snapshot().positions;
    return [for (index in motion.jointIndices) positions.get(index)];
  }

  public function resumePrepared(continuation:MachiningContinuation):Void {
    if (spindleFaulted) throw "cannot resume after spindle fault";
    var snapshot = motion.robot.snapshot();
    if (motion.running || motion.sessionState() != SessionState.Idle ||
        snapshot.trajectoryActive || snapshot.trajectoryQueueDepth > 0 ||
        snapshot.safety != 0)
      throw "machining restart needs a controlled stop";
    motion.run(continuation.program);
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

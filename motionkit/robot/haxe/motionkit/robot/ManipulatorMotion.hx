package motionkit.robot;

import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.program.InputPredicate;
import motionkit.program.MotionProgram;
import robotkit.world.FiredProcessEvent;
import robotkit.world.Robot;
import motionkit.robot.SessionState;

/** Runs compiled manipulator blocks and evaluates host-side barriers. */
class ManipulatorMotion {
  public final robot:Robot;
  public final compiler:ProgramCompiler;
  public var completed(get, never):Bool;
  public var failure(default, null):Null<String> = null;
  public var running(get, never):Bool;
  public final session:MotionSession = new MotionSession();
  final input:String -> Null<EventValue>;
  final eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool};
  final executor:PlanExecutor;
  public final jointIndices:Array<Int>;
  var compiled:Null<CompiledProgram>;
  var blockIndex:Int = 0;
  var planIndex:Int = 0;
  var barrierElapsed:Float = 0.0;
  var pendingProgram:Null<MotionProgram>;
  var programCompleted:Bool = false;
  var planStarted:Bool = false;
  var nextPlanId:Int64 = Int64.ofInt(1);
  var events:Array<FiredProcessEvent> = [];
  var lastCommandedQ:Null<Array<Float>> = null;

  public function new(robot:Robot, compiler:ProgramCompiler,
      input:String -> Null<EventValue>,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool},
      ?jointIndices:Array<Int>) {
    if (robot == null || compiler == null || input == null || eventSource == null)
      throw "ManipulatorMotion needs a robot, compiler, input and event source";
    this.robot = robot; this.compiler = compiler;
    this.input = input; this.eventSource = eventSource;
    var count = robot.description().joints.length;
    this.jointIndices = jointIndices == null ? [for (i in 0...count) i] :
      jointIndices.copy();
    if (this.jointIndices.length != compiler.solver.jointCount())
      throw "ManipulatorMotion joint count does not match compiler";
    var seen = new Map<Int, Bool>();
    for (index in this.jointIndices) {
      if (index < 0 || index >= count || seen.exists(index))
        throw "ManipulatorMotion needs distinct robot joint indices";
      seen.set(index, true);
    }
    executor = new PlanExecutor(robot, this.jointIndices, session);
  }

  function get_running():Bool return session.isActive();
  function get_completed():Bool return programCompleted;
  public function sessionState():SessionState return session.state;

  public function run(program:MotionProgram):Void {
    session.requireReady();
    if (program == null) throw "Manipulator program is required";
    if (session.isStopping()) {
      if (pendingProgram != null) throw "Manipulator program is already pending";
      pendingProgram = program;
      return;
    }
    if (running) throw "Manipulator program is already running";
    release(); programCompleted = false; failure = null; events = [];
    blockIndex = 0; planIndex = 0; barrierElapsed = 0.0; planStarted = false;
    try {
      var positions = robot.snapshot().positions;
      compiled = compiler.compile(program, lastCommandedQ == null
        ? [for (index in jointIndices) positions.get(index)]
        : lastCommandedQ.copy(), nextPlanId);
      for (block in compiled.blocks) nextPlanId = Int64.add(nextPlanId,
        Int64.ofInt(block.plans.length));
      session.begin();
      advance(0.0);
    } catch (error:Dynamic) { fail(Std.string(error), false); }
  }

  public function update(dtSeconds:Float):Void {
    if (!running) return;
    if (!Math.isFinite(dtSeconds) || dtSeconds <= 0.0)
      throw "Manipulator update duration must be finite and positive";
    try {
      collectEvents();
      if (session.isFaulted()) return;
      var fault = robot.fault();
      if (fault != null) {
        session.reject();
        throw 'Robot fault ${fault.code}: ${fault.message}';
      }
      executor.update();
      if (session.isStopping()) return;
      if (session.state == Idle) {
        release();
        var next = pendingProgram;
        pendingProgram = null;
        if (next != null) run(next);
        return;
      }
      if (session.isHolding()) return;
      if (executor.completed && planStarted) { planIndex++; planStarted = false; }
      advance(dtSeconds);
    } catch (error:Dynamic) { fail(Std.string(error), !session.isFaulted()); }
  }

  public function hold():Void if (running) executor.hold();
  public function resume():Void if (running) executor.resume();
  public function abort():Void {
    if (!running) return;
    pendingProgram = null;
    fail("Program aborted", true);
  }
  public function reset():Void {
    executor.reset();
    if (session.state == Idle) {
      release();
      pendingProgram = null;
      lastCommandedQ = null;
    }
  }
  public function firedEvents():Array<FiredProcessEvent> {
    collectEvents();
    return events.copy();
  }
  public function progress():ManipulatorProgress {
    var source = compiled;
    if (source == null || blockIndex >= source.blocks.length)
      return new ManipulatorProgress(blockIndex, -1, 0.0);
    var block = source.blocks[blockIndex];
    var op = planIndex < block.opIndices.length ? block.opIndices[planIndex] : -1;
    var distance = 0.0;
    if (planIndex < block.plans.length) {
      var plan = block.plans[planIndex];
      var times = block.pathTimes[planIndex];
      var distances = block.pathDistances[planIndex];
      if (times.length > 1) {
        distance = distances[distances.length - 1];
        for (index in 1...times.length)
          if (executor.elapsedSeconds <= times[index]) {
            var fraction = (executor.elapsedSeconds - times[index - 1]) /
              (times[index] - times[index - 1]);
            distance = distances[index - 1] + fraction *
              (distances[index] - distances[index - 1]);
            break;
          }
      }
    }
    return new ManipulatorProgress(blockIndex, op, distance,
      planIndex < block.plans.length ? block.plans[planIndex].guarantees() : null);
  }

  function advance(dt:Float):Void {
    var source = compiled;
    if (source == null) return;
    while (session.state == Running && blockIndex < source.blocks.length) {
      var block = source.blocks[blockIndex];
      if (planIndex < block.plans.length) {
        if (!planStarted) {
          executor.start(block.plans[planIndex], true);
          planStarted = true;
        }
        return;
      }
      if (block.barrier != null) {
        var ready = switch block.barrier {
          case Dwell(seconds):
            barrierElapsed += dt;
            barrierElapsed >= seconds;
          case WaitInput(channel, predicate, timeoutSeconds):
            var current = input(channel);
            var matches = current != null && switch predicate {
              case Equals(expected): switch [current, expected] {
                case [Digital(a), Digital(b)]: a == b;
                case [Analog(a), Analog(b)]: a == b;
                case [Process(a, x), Process(b, y)]: a == b && x == y;
                case _: false;
              };
            };
            if (!matches) {
              barrierElapsed += dt;
              if (timeoutSeconds != null && barrierElapsed >= timeoutSeconds)
                throw 'WaitInput "$channel" timed out after $timeoutSeconds seconds';
            }
            matches;
        };
        if (!ready) return;
      }
      blockIndex++; planIndex = 0; barrierElapsed = 0.0;
    }
    if (session.state == Running) {
      for (block in source.blocks)
        for (plan in block.plans)
          lastCommandedQ = plan.evaluate(plan.durationSeconds).positions;
      programCompleted = true; session.completed(); release();
    }
  }

  function hasPlan():Bool {
    var source = compiled;
    return source != null && blockIndex < source.blocks.length &&
      planIndex < source.blocks[blockIndex].plans.length;
  }
  function fail(message:String, stop:Bool):Void {
    if (stop && running && !session.isFaulted()) {
      try executor.abort() catch (error:Dynamic) {
        session.reject();
        message += ': ' + Std.string(error);
      }
    }
    failure = message; programCompleted = false;
    pendingProgram = null;
    lastCommandedQ = null;
    if (session.isFaulted()) { pendingProgram = null; release(); executor.clear(); }
    else if (session.state == Idle) release();
  }
  function collectEvents():Void {
    var batch = eventSource();
    for (event in batch.events) events.push(event);
    if (batch.overflow) {
      if (running) fail("Runtime process event overflow", true);
      else { failure = "Runtime process event overflow"; programCompleted = false; }
    }
  }
  function release():Void {
    if (compiled != null) { compiled.dispose(); compiled = null; }
  }
}

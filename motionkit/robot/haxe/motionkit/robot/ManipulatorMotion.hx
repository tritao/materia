package motionkit.robot;

import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.program.InputPredicate;
import motionkit.program.MotionProgram;
import motionkit.trajectory.ExecutionPlan;
import robotkit.world.FiredProcessEvent;
import robotkit.world.Robot;
import motionkit.robot.SessionState;

/**
  Runs manipulator programs and evaluates host-side barriers. A program is
  planned as it runs, on a worker thread about `LOOKAHEAD_SECONDS` of motion
  ahead of the machine, so neither starting a long program nor running it
  waits for planning; only if execution catches up with planning does an
  update wait for the next plan.
**/
class ManipulatorMotion {
  public final robot:Robot;
  public final compiler:ProgramCompiler;
  public var completed(get, never):Bool;
  public var failure(default, null):Null<String> = null;
  public var running(get, never):Bool;
  public final session:MotionSession = new MotionSession();
  /** What the plan checks found in the plans this motion has started, if its compiler runs one. */
  public final checks:PlanCheckSummary = new PlanCheckSummary();
  final input:String -> Null<EventValue>;
  final eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool};
  final executor:PlanExecutor;
  public final jointIndices:Array<Int>;
  /** Seconds of planned motion kept ahead of the machine. */
  public static inline final LOOKAHEAD_SECONDS = 1.0;
  var planner:Null<ProgramPlanner>;
  /** Plans of the running program started so far. */
  var startedPlans:Int = 0;
  var blockIndex:Int = 0;
  var planIndex:Int = 0;
  var barrierElapsed:Float = 0.0;
  var pendingProgram:Null<MotionProgram>;
  var programCompleted:Bool = false;
  var planStarted:Bool = false;
  var nextPlanId:Int64 = Int64.ofInt(1);
  var events:Array<FiredProcessEvent> = [];
  var lastCommandedQ:Null<Array<Float>> = null;
  /** Speed of every path, as a fraction of its programmed speed. */
  public var speedOverride(default, null):Float = 1.0;

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

  /**
   * The joint positions the last program commanded at its end, which the next program is planned
   * from; null before any program has finished and after a failure, when the next one starts from
   * where the robot is.
   */
  public function commandedPositions():Null<Array<Float>>
    return lastCommandedQ == null ? null : lastCommandedQ.copy();

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
    blockIndex = 0; planIndex = 0; barrierElapsed = 0.0; planStarted = false; startedPlans = 0;
    try {
      var positions = robot.snapshot().positions;
      var started = new ProgramPlanner(compiler, program, lastCommandedQ == null
        ? [for (index in jointIndices) positions.get(index)]
        : lastCommandedQ.copy(), nextPlanId, 0, speedOverride, LOOKAHEAD_SECONDS);
      planner = started;
      // Wait for the first motion or barrier, so a program that cannot start
      // fails here, before the session begins.
      while (!started.blocks.hasWork()) waitForPlans(started);
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
      var planning = planner;
      if (planning != null) {
        planning.poll();
        var problem = planning.failure;
        if (problem != null) throw problem;
      }
      if (executor.completed && planStarted) { planIndex++; planStarted = false; }
      advance(dtSeconds);
    } catch (error:Dynamic) { fail(Std.string(error), !session.isFaulted()); }
  }

  /**
    Sets the speed override, between 5% and 200% of the programmed speed. A
    running program takes it from the motion it has not planned yet, about
    `LOOKAHEAD_SECONDS` ahead, within the joint limits and without stopping.
  **/
  public function setSpeedOverride(scale:Float):Void {
    if (!Math.isFinite(scale) || scale < 0.05 || scale > 2.0)
      throw "Speed override must be between 0.05 and 2";
    speedOverride = scale;
    var planning = planner;
    if (planning != null) planning.setSpeedScale(scale);
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
    var planning = planner;
    if (planning == null || blockIndex >= planning.blocks.blocks.length)
      return new ManipulatorProgress(blockIndex, -1, 0.0);
    var block = planning.blocks.blocks[blockIndex];
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
    var planning = planner;
    if (planning == null) return;
    var source = planning.blocks;
    while (session.state == Running && blockIndex < source.blocks.length) {
      var block = source.blocks[blockIndex];
      if (planIndex < block.plans.length) {
        if (!planStarted) {
          var checked = block.plans[planIndex].checked;
          if (checked != null) checks.add(checked);
          executor.start(block.plans[planIndex], true);
          planStarted = true;
          startedPlans++;
          planning.started(startedPlans);
        }
        return;
      }
      // The machine caught up with planning: wait for the next plan.
      if (!block.complete) {
        waitForPlans(planning);
        continue;
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
        if (block.plans.length > 0) {
          var last = block.plans[block.plans.length - 1];
          lastCommandedQ = last.evaluate(last.durationSeconds).positions;
        }
      var next = source.nextPlanId;
      if (next != null) nextPlanId = next;
      programCompleted = true; session.completed(); release();
    }
  }

  /** Waits for the worker to plan more; throws its failure. */
  function waitForPlans(planning:ProgramPlanner):Void {
    if (planning.isStopped() && !planning.poll()) {
      var problem = planning.failure;
      throw problem != null ? problem : "Program planning stopped before the program was planned";
    }
    planning.waitForMore();
    nextPlanId = planning.nextPlanId;
    var problem = planning.failure;
    if (problem != null) throw problem;
  }

  function hasPlan():Bool {
    var planning = planner;
    return planning != null && blockIndex < planning.blocks.blocks.length &&
      planIndex < planning.blocks.blocks[blockIndex].plans.length;
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
    var planning = planner;
    if (planning != null) {
      nextPlanId = planning.nextPlanId;
      planning.dispose();
      planner = null;
    }
  }
}

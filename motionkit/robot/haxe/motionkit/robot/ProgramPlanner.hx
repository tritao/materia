package motionkit.robot;

import haxe.Int64;
import motionkit.program.MotionProgram;
import motionkit.trajectory.ExecutionPlan;
import sys.thread.Condition;
import sys.thread.Mutex;
import sys.thread.Thread;

private enum PlannedStep {
  Planned(plan:ExecutionPlan, opIndex:Int, length:Float, distances:Array<Float>,
    times:Array<Float>);
  Barrier(barrier:ProgramBarrier);
  Note(text:String);
  Finished(nextPlanId:Int64);
  Failed(message:String);
}

/**
  Plans a program on a worker thread, `lookaheadSeconds` of motion ahead of the
  plans that have started executing, and hands each step to the frame thread,
  which `poll`s them into `blocks`. The worker owns the compilation; the frame
  thread owns `blocks` and every plan in it, so neither touches the other's
  objects. A speed scale set while planning applies to motion not yet planned.

  A worker is inside MotionKit's native library while it plans, and `dispose` returns at once without waiting for it:
  the worker finishes the plan it is on and stops. A process that exits while workers are still planning runs the
  native library's static destructors under them, which crashes the worker (a use after destruction of the
  library's handle tables, seen as a segmentation fault or double free when a program exited just after a weld
  that gave up mid-plan). Whoever ends the process, or tears a world down, calls `shutdown` first.
**/
class ProgramPlanner {
  /** Planners whose worker has not yet stopped, under `registry`. */
  static final live:Array<ProgramPlanner> = [];
  static final registry = new Mutex();

  /** What is planned so far; frame thread only. */
  public final blocks:ProgramBlocks = new ProgramBlocks();
  /** Why planning stopped, if it failed. */
  public var failure(default, null):Null<String> = null;
  /** The next unused plan id: one past the last plan delivered. */
  public var nextPlanId(default, null):Int64;
  final lookaheadSeconds:Float;
  final condition = new Condition();
  // Shared with the worker, under `condition`.
  var outbox:Array<PlannedStep> = [];
  final plannedSeconds:Array<Float> = [];
  var startedPlans:Int = 0;
  var speedScale:Float;
  var cancelled:Bool = false;
  var stopped:Bool = false;
  var planningSeconds:Float = 0.0;
  var numericIkSolves:Int = 0;

  /** Worker CPU-phase wall time, excluding lookahead waits, and numeric pose queries. */
  public function planningMetrics():{seconds:Float, numericIkSolves:Int} {
    condition.acquire();
    var result = {seconds: planningSeconds, numericIkSolves: numericIkSolves};
    condition.release();
    return result;
  }

  /** Starts planning; an invalid program or start position throws here, on the caller's thread. */
  public function new(compiler:ProgramCompiler, program:MotionProgram, initialQ:Array<Float>,
      firstPlanId:Int64, firstOp:Int, speedScale:Float, lookaheadSeconds:Float) {
    if (!Math.isFinite(lookaheadSeconds) || lookaheadSeconds <= 0.0)
      throw "Planning lookahead must be finite and positive";
    this.lookaheadSeconds = lookaheadSeconds;
    this.speedScale = speedScale;
    nextPlanId = firstPlanId;
    // The worker plans on kinematics of its own: the caller keeps evaluating the arm meanwhile.
    var compilation = compiler.forWorker().begin(program, initialQ, firstPlanId, firstOp, speedScale,
      new WorkerSink(this));
    var self = this;
    registry.acquire();
    live.push(this);
    registry.release();
    Thread.create(function() self.work(compilation));
  }

  /** How many planners have a worker that has not stopped yet. */
  public static function active():Int {
    registry.acquire();
    var count = live.length;
    registry.release();
    return count;
  }

  /**
   * Cancels every planner and waits for its worker to stop, up to `timeoutSeconds` in all, so that no worker is inside
   * the native library when the process exits or the world is torn down. Returns how many workers were still running
   * when the time ran out (none, normally).
   */
  public static function shutdown(timeoutSeconds:Float = 30.0):Int {
    registry.acquire();
    var planners = live.copy();
    registry.release();
    for (planner in planners) planner.dispose();
    var polls = Std.int(Math.ceil(timeoutSeconds / 0.005));
    while (active() > 0 && polls-- > 0) Sys.sleep(0.005);
    return active();
  }

  /** Moves what the worker has planned since the last poll into `blocks`; true if anything came. */
  public function poll():Bool {
    condition.acquire();
    var steps = outbox;
    outbox = [];
    condition.release();
    for (step in steps) apply(step);
    return steps.length > 0;
  }

  /** Waits until the worker delivers more or stops, then polls. */
  public function waitForMore():Void {
    condition.acquire();
    while (outbox.length == 0 && !stopped) condition.wait();
    condition.release();
    poll();
  }

  /** Whether the worker has stopped: the program is planned, planning failed, or it was cancelled. */
  public function isStopped():Bool {
    condition.acquire();
    var value = stopped;
    condition.release();
    return value;
  }

  /** Execution has started `count` plans in all; the worker stays `lookaheadSeconds` past them. */
  public function started(count:Int):Void {
    condition.acquire();
    startedPlans = count;
    condition.broadcast();
    condition.release();
  }

  public function setSpeedScale(scale:Float):Void {
    if (!Math.isFinite(scale) || scale <= 0.0)
      throw "Program speed scale must be finite and positive";
    condition.acquire();
    speedScale = scale;
    condition.release();
  }

  /** Stops the worker and disposes every plan, delivered or not. Returns at once. */
  public function dispose():Void {
    condition.acquire();
    cancelled = true;
    var steps = outbox;
    outbox = [];
    condition.broadcast();
    condition.release();
    disposeSteps(steps);
    blocks.dispose();
  }

  function apply(step:PlannedStep):Void {
    switch step {
      case Planned(plan, opIndex, length, distances, times):
        blocks.plan(plan, opIndex, length, distances, times);
        nextPlanId = Int64.add(plan.planId, Int64.ofInt(1));
      case Barrier(barrier): blocks.barrier(barrier);
      case Note(text): blocks.note(text);
      case Finished(next):
        blocks.finish(next);
        nextPlanId = next;
      case Failed(message): failure = message;
    }
  }

  /** The worker: plans while less than the lookahead waits beyond the started plans. */
  function work(compilation:ProgramCompilation):Void {
    try {
      while (true) {
        condition.acquire();
        while (!cancelled && secondsAhead() >= lookaheadSeconds) condition.wait();
        var cancel = cancelled;
        compilation.speedScale = speedScale;
        condition.release();
        if (cancel) break;
        var numeric:Null<ManipulatorKinematics> = Std.isOfType(compilation.compiler.solver, ManipulatorKinematics)
          ? cast compilation.compiler.solver : null;
        var before = numeric == null ? 0 : numeric.manipulator.numericSolveCount();
        var began = Sys.time();
        var more = compilation.step();
        var elapsed = Sys.time() - began;
        var solves = numeric == null ? 0 : numeric.manipulator.numericSolveCount() - before;
        condition.acquire();
        planningSeconds += elapsed;
        numericIkSolves += solves;
        condition.release();
        if (!more) break;
      }
    } catch (error:Dynamic) {
      deliver(Failed(Std.string(error)));
    }
    compilation.dispose();
    condition.acquire();
    stopped = true;
    condition.broadcast();
    condition.release();
    registry.acquire();
    live.remove(this);
    registry.release();
  }

  /** Seconds of planned motion after the started plans; call under `condition`. */
  function secondsAhead():Float {
    var ahead = 0.0;
    for (index in startedPlans...plannedSeconds.length) ahead += plannedSeconds[index];
    return ahead;
  }

  /** Worker side: queues a step for the frame thread, or disposes it once cancelled. */
  @:allow(motionkit.robot.WorkerSink)
  function deliver(step:PlannedStep):Void {
    condition.acquire();
    if (cancelled) {
      condition.release();
      disposeSteps([step]);
      return;
    }
    switch step {
      case Planned(plan, _, _, _, _): plannedSeconds.push(plan.durationSeconds);
      case _:
    }
    outbox.push(step);
    condition.broadcast();
    condition.release();
  }

  static function disposeSteps(steps:Array<PlannedStep>):Void
    for (step in steps) switch step {
      case Planned(plan, _, _, _, _): plan.dispose();
      case _:
    }
}

/** The worker's sink: each step goes to the frame thread through the planner. */
private class WorkerSink implements ProgramSink {
  final planner:ProgramPlanner;

  public function new(planner:ProgramPlanner) this.planner = planner;

  public function plan(plan:ExecutionPlan, opIndex:Int, length:Float, distances:Array<Float>,
      times:Array<Float>):Void
    planner.deliver(Planned(plan, opIndex, length, distances, times));

  public function barrier(barrier:ProgramBarrier):Void planner.deliver(Barrier(barrier));

  public function note(text:String):Void planner.deliver(Note(text));

  public function finish(nextPlanId:Int64):Void planner.deliver(Finished(nextPlanId));
}

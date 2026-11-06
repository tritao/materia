package machinekit.cache;

import sys.thread.Lock;
import sys.thread.Mutex;
import sys.thread.Thread;

/**
 * Independent build steps run side by side on a few worker threads, with results that do not depend on the scheduling:
 * every task's result lands at its task's index, and an error is the one the first failing task (in task order) threw,
 * after all have finished. Tasks must share nothing they change except through `Memo`/`Once` (which are thread-safe) and
 * must only read native shapes they did not build (see `Memo`).
 *
 * `MACHINEKIT_THREADS` caps the threads working at once, the caller included (default: the machine's cores, at most
 * `MAX_THREADS`); `MACHINEKIT_THREADS=1` runs every step serially on the calling thread, in task order, for debugging.
 * Steps nest: a task may start a step of its own, which takes helper threads while the cap allows and otherwise runs in
 * that task's thread. A step's caller always works through its own tasks too, so a step never waits on one it started.
 */
class Parallel {
  /** More threads than this do not pay for the build steps there are (two foundations, a handful of trials). */
  static inline final MAX_THREADS = 8;

  static final threadLock = new Mutex();
  static var threadCount = 0;
  /** Helper threads running now, across all steps. */
  static var helpers = 0;

  /** The workers a step may use (1: serial). */
  public static function threads():Int {
    threadLock.acquire();
    if (threadCount == 0) {
      var configured = Sys.getEnv("MACHINEKIT_THREADS");
      var parsed = configured == null ? null : Std.parseInt(configured);
      threadCount = parsed != null && parsed >= 1 ? parsed : Std.int(Math.min(MAX_THREADS, cores()));
    }
    var result = threadCount;
    threadLock.release();
    return result;
  }

  /** Overrides `MACHINEKIT_THREADS` for the rest of the process (tests compare serial and parallel runs). */
  public static function setThreads(count:Int):Void {
    if (count < 1) throw "Parallel needs at least one thread";
    threadLock.acquire();
    threadCount = count;
    threadLock.release();
  }

  static function cores():Int {
    try {
      var count = 0;
      for (line in sys.io.File.getContent("/proc/cpuinfo").split("\n")) if (StringTools.startsWith(line, "processor")) count++;
      return count < 1 ? 1 : count;
    } catch (_:Dynamic) {
      return 1;
    }
  }

  /** Native work that must not run on two threads at once, one call at a time process-wide. */
  static final serialLock = new Mutex();

  /**
   * `work` run while no other thread runs serialized work: OCCT's STEP translator keeps process-wide state (its static
   * parameters and session), so every STEP import goes through here.
   */
  public static function serialized<T>(work:() -> T):T {
    serialLock.acquire();
    try {
      var result = work();
      serialLock.release();
      return result;
    } catch (error:Dynamic) {
      serialLock.release();
      throw error;
    }
  }

  /** Every task's result, in task order. */
  public static function all<T>(tasks:Array<() -> T>):Array<T> {
    var results:Array<Null<T>> = [], errors:Array<Dynamic> = [], failed:Array<Bool> = [];
    for (_ in tasks) {
      results.push(null);
      errors.push(null);
      failed.push(false);
    }
    var extra = claim(tasks.length - 1);
    if (extra == 0) {
      for (index in 0...tasks.length) results[index] = tasks[index]();
      return cast results;
    }
    var queue = new Mutex(), next = 0, finished = new Lock();
    function work():Void {
      while (true) {
        queue.acquire();
        var index = next++;
        queue.release();
        if (index >= tasks.length) break;
        try results[index] = tasks[index]() catch (error:Dynamic) {
          errors[index] = error;
          failed[index] = true;
        }
      }
    }
    for (_ in 0...extra) Thread.create(() -> {
      work();
      threadLock.acquire();
      helpers--;
      threadLock.release();
      finished.release();
    });
    // The calling thread takes tasks too, so the step finishes even when every other thread is busy.
    work();
    for (_ in 0...extra) finished.wait();
    for (index in 0...tasks.length) if (failed[index]) throw errors[index];
    return cast results;
  }

  /** Up to `wanted` helper threads, within the cap: how many were granted. */
  static function claim(wanted:Int):Int {
    var cap = threads();
    threadLock.acquire();
    // The cap counts every working thread: the helpers and the threads that started steps (at least this one).
    var granted = Std.int(Math.max(0, Math.min(wanted, cap - 1 - helpers)));
    helpers += granted;
    threadLock.release();
    return granted;
  }

  /**
   * The first index below `count`, in order, whose `attempt` gives a value (null: that index does not do), with that value;
   * null when none does. Attempts run `threads()` at a time, in order, so a later attempt may run that the serial scan would
   * have skipped (its result is discarded); an attempt that throws stops the scan with that error, if no earlier index gave a
   * value, as the serial scan would.
   */
  public static function first<T>(count:Int, attempt:Int -> Null<T>):Null<ParallelFound<T>> {
    var batch = threads(), start = 0;
    if (batch <= 1) {
      for (index in 0...count) {
        var value = attempt(index);
        if (value != null) return new ParallelFound<T>(index, value);
      }
      return null;
    }
    while (start < count) {
      var end = Std.int(Math.min(count, start + batch));
      var tasks:Array<() -> ParallelAttempt<T>> = [];
      for (index in start...end) tasks.push(() -> {
        try return new ParallelAttempt<T>(attempt(index), null, false) catch (error:Dynamic) return new ParallelAttempt<T>(null, error, true);
      });
      var outcomes = all(tasks);
      for (offset in 0...outcomes.length) {
        var outcome = outcomes[offset];
        if (outcome.failed) throw outcome.error;
        if (outcome.value != null) return new ParallelFound<T>(start + offset, outcome.value);
      }
      start = end;
    }
    return null;
  }
}

/** What `Parallel.first` found: the index and its attempt's value. */
class ParallelFound<T> {
  public final index:Int;
  public final value:T;

  public function new(index:Int, value:T) {
    this.index = index;
    this.value = value;
  }
}

private class ParallelAttempt<T> {
  public final value:Null<T>;
  public final error:Dynamic;
  public final failed:Bool;

  public function new(value:Null<T>, error:Dynamic, failed:Bool) {
    this.value = value;
    this.error = error;
    this.failed = failed;
  }
}

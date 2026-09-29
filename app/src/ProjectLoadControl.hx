package app;

import sys.io.Process;
import sys.thread.Mutex;

/**
 * Shared between a project load running on a worker thread and the UI thread that watches it: the worker
 * reports the current phase and registers the child process, the UI reads the phase and can cancel.
 */
class ProjectLoadControl {
  /** Thrown by a load that was cancelled; callers treat it as a normal outcome, not a failure. */
  public static inline var CANCELLED:String = "project load cancelled";

  final mutex:Mutex = new Mutex();
  var phaseText:String = "Starting";
  var cancelRequested:Bool = false;
  var running:Null<Process> = null;

  public function new() {}

  public function phase(text:String):Void {
    mutex.acquire();
    phaseText = text;
    mutex.release();
  }

  public function currentPhase():String {
    mutex.acquire();
    var text = phaseText;
    mutex.release();
    return text;
  }

  public function isCancelled():Bool {
    mutex.acquire();
    var value = cancelRequested;
    mutex.release();
    return value;
  }

  /** Stops the load: the running child process is killed and the worker throws CANCELLED. */
  public function cancel():Void {
    mutex.acquire();
    cancelRequested = true;
    var process = running;
    mutex.release();
    if (process != null) try process.kill() catch (_:Dynamic) {}
  }

  public function throwIfCancelled():Void {
    if (isCancelled()) throw CANCELLED;
  }

  /** Called by the worker around each child process so cancel() can reach it. */
  public function attach(process:Process):Void {
    mutex.acquire();
    running = process;
    var stop = cancelRequested;
    mutex.release();
    if (stop) try process.kill() catch (_:Dynamic) {}
  }

  public function detach():Void {
    mutex.acquire();
    running = null;
    mutex.release();
  }
}

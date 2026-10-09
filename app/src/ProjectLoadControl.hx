package app;

#if !wasm
import sys.io.Process;
#end
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
  final startedAt:Float = Sys.time();
  var phaseStartedAt:Float = Sys.time();
  var measurements:Array<{name:String, milliseconds:Float}> = [];
  var timings:Array<{phase:String, milliseconds:Float}> = [];
  var cancelRequested:Bool = false;
  var running:Null<Void->Void> = null;

  public function new() {}

  public function phase(text:String):Void {
    mutex.acquire();
    var now = Sys.time();
    timings.push({phase: phaseText, milliseconds: (now - phaseStartedAt) * 1000.0});
    phaseStartedAt = now;
    phaseText = text;
    mutex.release();
  }

  public function measure(name:String, seconds:Float):Void {
    mutex.acquire();
    measurements.push({name: name, milliseconds: seconds * 1000.0});
    mutex.release();
  }

  public function profile():Dynamic {
    mutex.acquire();
    var values = timings.copy();
    values.push({phase: phaseText, milliseconds: (Sys.time() - phaseStartedAt) * 1000.0});
    var work = measurements.copy();
    mutex.release();
    return {measurements: work, totalMilliseconds: (Sys.time() - startedAt) * 1000.0, phases: values};
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
    if (process != null) try process() catch (_:Dynamic) {}
  }

  public function throwIfCancelled():Void {
    if (isCancelled()) throw CANCELLED;
  }

  /** Installs the execution adapter's cancellation action. Invoked outside the mutex. */
  public function attachCancellation(action:Void->Void):Void {
    mutex.acquire();
    running = action;
    var stop = cancelRequested;
    mutex.release();
    if (stop) try action() catch (_:Dynamic) {}
  }

  public function detach():Void {
    mutex.acquire();
    running = null;
    mutex.release();
  }

  #if !wasm
  /** Called by the worker around each child process so cancel() can reach it. */
  public function attach(process:Process):Void {
    mutex.acquire();
    running = function() process.kill();
    var stop = cancelRequested;
    mutex.release();
    if (stop) try process.kill() catch (_:Dynamic) {}
  }

  public function attachChild(process:sys.io.ChildProcess):Void {
    mutex.acquire();
    running = function() process.cancel();
    var stop = cancelRequested;
    mutex.release();
    if (stop) try process.cancel() catch (_:Dynamic) {}
  }

  #end
}

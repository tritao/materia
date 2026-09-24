package app;

import nativekit.ui.docking.DockWorkspacePersistence;
import nativekit.ui.docking.DockWorkspaceSnapshot;
import nativekit.ui.docking.DockWorkspaceSnapshotCodec;
import sys.thread.Condition;
import sys.thread.Thread;

/** Coalesces workspace changes and serializes/writes them away from the UI thread. */
class WorkspaceSaveWorker {
  static inline var QUIET_SECONDS = 0.15;

  final storage:DockWorkspacePersistence;
  final key:String;
  final state = new Condition();
  var pending:Null<DockWorkspaceSnapshot>;
  var requestedGeneration = 0;
  var completedGeneration = 0;
  var errorGeneration = 0;
  var reportedErrorGeneration = 0;
  var lastError:Null<String>;
  var urgent = false;
  var closing = false;
  var exited = false;

  public function new(storage:DockWorkspacePersistence, key:String) {
    this.storage = storage;
    this.key = key;
    Thread.create(run);
  }

  /** The snapshot is already detached from the live workspace model. */
  public function schedule(snapshot:DockWorkspaceSnapshot):Void {
    enqueue(snapshot, false);
  }

  /** Explicit Save waits for this snapshot to reach storage. */
  public function saveNow(snapshot:DockWorkspaceSnapshot):Null<String> {
    var generation = enqueue(snapshot, true);
    state.acquire();
    while (completedGeneration < generation && !exited)
      state.wait();
    var error = errorGeneration >= generation ? lastError : null;
    if (error != null) reportedErrorGeneration = errorGeneration;
    state.release();
    return error;
  }

  /** Returns each background failure once, for reporting on the UI thread. */
  public function takeError():Null<String> {
    state.acquire();
    var error = errorGeneration > reportedErrorGeneration ? lastError : null;
    reportedErrorGeneration = errorGeneration;
    state.release();
    return error;
  }

  /** Flushes pending work and joins the worker before the editor is destroyed. */
  public function close():Null<String> {
    state.acquire();
    closing = true;
    urgent = true;
    state.broadcast();
    while (!exited)
      state.wait();
    var error = lastError;
    state.release();
    return error;
  }

  function enqueue(snapshot:DockWorkspaceSnapshot, immediate:Bool):Int {
    if (snapshot == null) throw "Cannot save a null workspace snapshot";
    state.acquire();
    if (closing) {
      state.release();
      throw "Workspace save worker is closed";
    }
    pending = snapshot;
    requestedGeneration++;
    if (immediate) urgent = true;
    var generation = requestedGeneration;
    state.broadcast();
    state.release();
    return generation;
  }

  function run():Void {
    while (true) {
      state.acquire();
      while (pending == null && !closing)
        state.wait();
      if (pending == null && closing) {
        exited = true;
        state.broadcast();
        state.release();
        return;
      }
      if (!urgent && !closing) {
        var observed = requestedGeneration;
        var deadline = Sys.time() + QUIET_SECONDS;
        while (!urgent && !closing && requestedGeneration == observed) {
          var remaining = deadline - Sys.time();
          if (remaining <= 0) break;
          state.timedWait(remaining);
        }
        if (requestedGeneration != observed && !urgent && !closing) {
          state.release();
          continue;
        }
      }
      var snapshot = pending;
      var generation = requestedGeneration;
      pending = null;
      urgent = false;
      state.release();

      var error:Null<String> = null;
      try storage.save(key, DockWorkspaceSnapshotCodec.encode(snapshot));
      catch (failure:Dynamic) error = Std.string(failure);

      state.acquire();
      completedGeneration = generation;
      if (error != null) {
        lastError = error;
        errorGeneration = generation;
      } else {
        lastError = null;
      }
      state.broadcast();
      state.release();
    }
  }
}

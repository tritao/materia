package tests;

import app.WorkspaceSaveWorker;
import nativekit.ui.docking.DockNode;
import nativekit.ui.docking.DockWorkspacePersistence;
import nativekit.ui.docking.DockWorkspaceSnapshot;
import nativekit.ui.docking.DockWorkspaceSnapshotCodec;
import sys.thread.Condition;

class WorkspaceSaveWorkerTests {
  static function snapshot(id:String):DockWorkspaceSnapshot
    return new DockWorkspaceSnapshot(DockNode.Panel(id), id);

  public static function main():Int {
    var storage = new BlockingWorkspaceStorage();
    var worker = new WorkspaceSaveWorker(storage, "test");
    worker.schedule(snapshot("first"));
    storage.waitForFirstWrite();
    worker.schedule(snapshot("second"));
    worker.schedule(snapshot("latest"));
    storage.releaseWrites();
    var error = worker.close();
    if (error != null) throw "Workspace worker failed: " + error;
    if (storage.writes.length != 2)
      throw "Workspace worker did not coalesce queued changes";
    var saved = DockWorkspaceSnapshotCodec.decode(storage.writes[1]);
    if (saved == null || saved.activePanelId != "latest")
      throw "Workspace worker did not flush the latest snapshot";
    var explicit = new WorkspaceSaveWorker(storage, "test");
    if (explicit.saveNow(snapshot("explicit")) != null)
      throw "Explicit workspace save failed";
    explicit.close();
    var explicitSaved = DockWorkspaceSnapshotCodec.decode(storage.writes[2]);
    if (explicitSaved == null || explicitSaved.activePanelId != "explicit")
      throw "Explicit workspace save did not wait for storage";
    var failing = new WorkspaceSaveWorker(new FailingWorkspaceStorage(), "test");
    if (failing.saveNow(snapshot("failure")) != "disk full")
      throw "Workspace save failure was not returned to the UI thread";
    failing.close();
    return 0;
  }
}

private class FailingWorkspaceStorage implements DockWorkspacePersistence {
  public function new() {}
  public function load(key:String):Null<String> return null;
  public function save(key:String, value:String):Void throw "disk full";
}

private class BlockingWorkspaceStorage implements DockWorkspacePersistence {
  final state = new Condition();
  var firstWriteStarted = false;
  var writesReleased = false;
  public final writes:Array<String> = [];

  public function new() {}

  public function load(key:String):Null<String> return null;

  public function save(key:String, value:String):Void {
    state.acquire();
    firstWriteStarted = true;
    state.broadcast();
    while (!writesReleased)
      state.wait();
    writes.push(value);
    state.release();
  }

  public function waitForFirstWrite():Void {
    state.acquire();
    while (!firstWriteStarted)
      state.wait();
    state.release();
  }

  public function releaseWrites():Void {
    state.acquire();
    writesReleased = true;
    state.broadcast();
    state.release();
  }
}

package tests;

import app.EditorScene;
import app.SceneCodec;
import app.SceneDocumentSession;
import app.SceneDocumentController;
import app.SceneFileDialogs;
import app.EditorSceneViewport;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyValue;
import sys.FileSystem;
import sys.io.File;
import haxe.Json;
import nativekit.ui.core.ViewportCamera;

class SceneDocumentTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.00001, message);
  static function rejects(action:Void->Void, message:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, message);
  }
  static function edit(session:SceneDocumentSession, value:Float):Void {
    session.scene.select("box");
    new PropertyBinding(session.scene.properties()[0], session.scene.context()).apply(PropertyValue.Float(value));
  }

  public static function run():Void {
    var directory = "build/document-tests-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var first = directory + "/scene.materia.json";
    var second = directory + "/copy.materia.json";
    var bad = directory + "/invalid.materia.json";
    var session = new SceneDocumentSession();
    try {
      edit(session, 1.25);
      session.save(first);
      check(!session.scene.document.isDirty && session.path != null, "save establishes path and savepoint");
      var saved = File.getContent(first);
      edit(session, 2.5);
      check(session.scene.document.history.undoCount == 2, "edits cannot coalesce across a savepoint");
      session.scene.document.undo();
      near(session.scene.info("box").localTransform().element(12), 1.25, "undo returns to saved position");
      check(!session.scene.document.isDirty, "undo reaches clean savepoint");
      session.scene.document.undo();
      check(session.scene.document.isDirty, "undo before saved state is dirty");
      session.scene.document.redo();
      check(!session.scene.document.isDirty, "redo reaches saved state");
      edit(session, 3.5);
      check(session.scene.document.history.undoCount == 2, "new branch after undo does not merge through savepoint");
      var originalPath = session.path;
      rejects(function() session.save(first + "/cannot-write.json"), "failed write is reported");
      check(session.path == originalPath && session.scene.document.isDirty, "failed save retains path and dirty state");
      check(File.getContent(first) == saved, "failed save preserves original file");
      session.save(second);
      check(File.getContent(first) == saved, "Save As preserves original document");
      check(session.path != originalPath && !session.scene.document.isDirty, "Save As adopts new path");

      session.scene.select("tower");
      new PropertyBinding(session.scene.properties()[2], session.scene.context()).apply(PropertyValue.Bool(false));
      session.save(second);
      var encoded = SceneCodec.encode(session.scene);
      var oldRevision = session.scene.revision;
      session.newDocument();
      check(session.scene.revision > oldRevision, "new document invalidates retained viewport and tree caches");
      oldRevision = session.scene.revision;
      session.open(second);
      check(session.scene.revision > oldRevision, "opened document invalidates retained viewport and tree caches");
      check(SceneCodec.encode(session.scene) == encoded, "round trip preserves every serialized property");
      check(!session.scene.document.canUndo && !session.scene.document.isDirty, "opened document starts with clean history");
      check(!session.scene.info("tower").visible(), "hidden state is restored");
      check(session.scene.pick(1.1, 0) == "scene", "loaded hidden geometry is excluded from picking");

      var record:Dynamic = Json.parse(encoded);
      var records:Array<Dynamic> = cast Reflect.field(record, "objects");
      Reflect.setField(records[0], "id", "part/custom-id");
      Reflect.setField(records[0], "label", "Custom part");
      var loaded = new EditorScene(SceneCodec.decode(Json.stringify(record)));
      check(loaded.items()[0].id == "part/custom-id", "document IDs survive native scene reconstruction");
      check(loaded.items()[0].label == "Custom part", "object names restored");
      loaded.dispose();
      Reflect.setField(record, "version", 99);
      rejects(function() { SceneCodec.decode(Json.stringify(record)); }, "unsupported version rejected");
      Reflect.setField(record, "version", 1);
      Reflect.setField(records[1], "id", Reflect.field(records[0], "id"));
      rejects(function() { SceneCodec.decode(Json.stringify(record)); }, "duplicate IDs rejected");
      Reflect.setField(records[1], "id", "tower");
      Reflect.setField(records[0], "width", -1);
      rejects(function() { SceneCodec.decode(Json.stringify(record)); }, "negative dimensions rejected");
      Reflect.setField(records[0], "width", 1);
      Reflect.setField(records[0], "visible", "yes");
      rejects(function() { SceneCodec.decode(Json.stringify(record)); }, "wrong field types rejected");
      rejects(function() { SceneCodec.decode("null"); }, "null root rejected");
      rejects(function() { SceneCodec.decode('{"format":"materia.scene","version":1,"objects":null}'); }, "null objects rejected");
      var empty = new EditorScene(SceneCodec.decode('{"format":"materia.scene","version":1,"objects":[]}'));
      check(empty.items().length == 0 && empty.selectedId == "scene", "empty scene opens safely");
      empty.dispose();

      File.saveContent(bad, "{ broken JSON");
      edit(session, 4.0);
      var previousScene = session.scene;
      var previousHistory = session.scene.document.history.undoCount;
      rejects(function() session.open(bad), "invalid file fails to open");
      check(session.scene == previousScene && session.scene.document.history.undoCount == previousHistory,
        "invalid open preserves scene and history");
      rejects(function() session.open(directory + "/missing.json"), "missing file fails to open");

      var chosen:Null<String> = null;
      var chooserError:Null<String> = null;
      var controller = new SceneDocumentController(session, function(_, _, done) done(chosen, chooserError), function() {});
      controller.requestNew();
      check(controller.needsConfirmation(), "New asks before discarding edits");
      controller.resolve("cancel");
      check(session.scene == previousScene && session.scene.document.isDirty, "Cancel preserves document");
      controller.requestNew();
      controller.resolve("discard");
      check(session.scene != previousScene && session.path == null && !session.scene.document.isDirty,
        "Discard creates a fresh untitled document");
      edit(session, 5);
      var closed = false;
      controller.requestClose(function() closed = true);
      controller.resolve("save");
      check(!closed && session.scene.document.isDirty, "cancelled save chooser does not close");
      chosen = first + "/impossible";
      controller.requestClose(function() closed = true);
      controller.resolve("save");
      check(!closed && controller.error != null && session.scene.document.isDirty, "failed save does not close");
      controller.dismissError();
      chosen = second;
      controller.requestClose(function() closed = true);
      controller.resolve("save");
      check(closed && !session.scene.document.isDirty, "successful save continues close request");
      edit(session, 6);
      chosen = bad;
      controller.requestOpen();
      controller.resolve("discard");
      check(controller.error != null && session.scene.document.isDirty, "failed open after Discard retains document");
      controller.dismissError();
      chosen = first;
      controller.requestOpen();
      controller.resolve("discard");
      near(session.scene.info("box").localTransform().element(12), 1.25, "Open loads chosen scene");
      check(!controller.blocked() && !session.scene.document.isDirty, "Open resets workflow and dirty state");

      edit(session, -2.0);
      session.save(first);
      var historyBeforeDrag = session.scene.document.history.undoCount;
      var viewport = new EditorSceneViewport(session.scene);
      var camera = new ViewportCamera(1.8, 25, -15);
      var commitCount = 0;
      var cancelCount = 0;
      var boundaryController = new SceneDocumentController(session,
        function(_, _, done) done(null, null), function() {}, function() {
          if (viewport.commitDrag()) commitCount++;
        }, function() {
          if (viewport.cancelDrag()) cancelCount++;
        });
      var start = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 2.0 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y);
      var finish = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 1.0 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.5 * EditorSceneViewport.SCALE);
      check(viewport.beginDrag(camera, start.x, start.y, false), "document-boundary drag begins");
      viewport.updateDrag(camera, finish.x, finish.y);
      check(!session.scene.document.isDirty, "active drag preview remains outside saved history");
      boundaryController.save();
      check(commitCount == 1 && !viewport.dragging(), "Save commits the active drag");
      check(!session.scene.document.isDirty &&
        session.scene.document.history.undoCount == historyBeforeDrag + 1,
        "Save establishes a savepoint after the committed move");
      session.scene.document.undo();
      near(session.scene.info("box").localTransform().element(12), -2.0,
        "undo after Save restores the pre-drag position");
      check(session.scene.document.isDirty, "undo before the drag savepoint is dirty");
      session.scene.document.redo();
      near(session.scene.info("box").localTransform().element(12), -1.0,
        "redo after Save restores the persisted drag");
      check(!session.scene.document.isDirty, "redo returns to the drag savepoint");

      edit(session, -2.5);
      var dirtyStart = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 2.5 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.5 * EditorSceneViewport.SCALE);
      var dirtyFinish = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 1.5 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 1.0 * EditorSceneViewport.SCALE);
      viewport.beginDrag(camera, dirtyStart.x, dirtyStart.y, false);
      viewport.updateDrag(camera, dirtyFinish.x, dirtyFinish.y);
      boundaryController.requestNew();
      check(cancelCount == 1 && !viewport.dragging() && boundaryController.needsConfirmation(),
        "New cancels drag before starting the unsaved-change flow");
      near(session.scene.info("box").localTransform().element(12), -2.5,
        "New interruption restores the pre-drag position");
      boundaryController.resolve("cancel");

      viewport.beginDrag(camera, dirtyStart.x, dirtyStart.y, false);
      viewport.updateDrag(camera, dirtyFinish.x, dirtyFinish.y);
      boundaryController.requestOpen();
      check(cancelCount == 2 && boundaryController.needsConfirmation(),
        "Open cancels drag before starting the unsaved-change flow");
      boundaryController.resolve("cancel");
      var boundaryClosed = false;
      viewport.beginDrag(camera, dirtyStart.x, dirtyStart.y, false);
      viewport.updateDrag(camera, dirtyFinish.x, dirtyFinish.y);
      boundaryController.requestClose(function() boundaryClosed = true);
      check(cancelCount == 3 && boundaryController.needsConfirmation() && !boundaryClosed,
        "Close cancels drag before starting the unsaved-change flow");
      boundaryController.resolve("cancel");

      check(SceneFileDialogs.localPath("file:///tmp/a%20b+c.json") == "/tmp/a b+c.json", "file URI decoding preserves plus");
      check(SceneFileDialogs.localPath("file://localhost/tmp/a.json") == "/tmp/a.json", "localhost URI accepted");
      rejects(function() { SceneFileDialogs.localPath("file://remote/tmp/a"); }, "remote URI rejected");
      rejects(function() { SceneFileDialogs.localPath("file:///tmp/%ZZ"); }, "invalid URI escape rejected");
      rejects(function() { SceneFileDialogs.localPath("file:///tmp/%00"); }, "NUL URI rejected");
      session.dispose();
      FileSystem.deleteFile(first);
      FileSystem.deleteFile(second);
      FileSystem.deleteFile(bad);
      FileSystem.deleteDirectory(directory);
      Sys.println("Scene document tests passed");
    } catch (error:Dynamic) {
      session.dispose();
      throw error;
    }
  }
}

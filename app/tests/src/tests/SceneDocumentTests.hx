package tests;

import app.EditorScene;
import app.SceneCodec;
import app.ProjectDocumentSession;
import app.SceneDocumentController;
import app.SceneFileDialogs;
import app.PerspectiveCamera;
import app.PerspectiveSceneDrag;
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyValue;
import sys.FileSystem;
import sys.io.File;
import haxe.Json;

class SceneDocumentTests {
  static function property(properties:Array<PropertyDescriptor>, key:String):PropertyDescriptor {
    for (candidate in properties)
      if (StringTools.endsWith(candidate.id, ":" + key)) return candidate;
    throw "Missing property: " + key;
  }
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
  static function edit(session:ProjectDocumentSession, value:Float):Void {
    session.scene.select("box");
    new PropertyBinding(property(session.scene.properties(), "position-0"), session.scene.context()).apply(PropertyValue.Float(value));
  }

  public static function run():Void {
    var directory = "build/document-tests-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var first = directory + "/scene.materia.json";
    var second = directory + "/copy.materia.json";
    var bad = directory + "/invalid.materia.json";
    var codeOwned = directory + "/code-owned.materia.json";
    var session = new ProjectDocumentSession();
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
      new PropertyBinding(property(session.scene.properties(), "visible"), session.scene.context()).apply(PropertyValue.Bool(false));
      session.save(second);
      var encoded = SceneCodec.encode(session.scene, session.sensors);
      var oldRevision = session.scene.revision;
      session.newDocument();
      check(session.scene.revision > oldRevision, "new document invalidates retained viewport and tree caches");
      oldRevision = session.scene.revision;
      session.open(second);
      check(session.scene.revision > oldRevision, "opened document invalidates retained viewport and tree caches");
      check(SceneCodec.encode(session.scene, session.sensors) == encoded, "round trip preserves every serialized property");
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
      File.saveContent(codeOwned, '{"project":{"version":1,"reference":"missing-project.py",' +
        '"overrides":[],"removed":[],"instances":[]}}');
      chosen = codeOwned;
      var beforeTrust = session.scene;
      controller.requestOpen();
      check(controller.needsTrustConfirmation() && controller.trustReference == "missing-project.py" &&
        session.scene == beforeTrust, "project code is not run before trust confirmation");
      controller.resolveTrust(false);
      check(!controller.blocked() && session.scene == beforeTrust,
        "cancelling project code leaves the document open");
      controller.requestOpen();
      controller.resolveTrust(true);
      check(controller.error != null && session.scene == beforeTrust,
        "trust confirmation reaches project loading without replacing on failure");
      controller.dismissError();
      chosen = first;

      edit(session, -2.0);
      session.save(first);
      var historyBeforeDrag = session.scene.document.history.undoCount;
      var camera = new PerspectiveCamera();
      camera.frame(-2.0, 0.0, 0.0, 1.6, 1.2, 0.05, 4.0 / 3.0);
      var drag:Null<PerspectiveSceneDrag> = null;
      var commitCount = 0;
      var cancelCount = 0;
      var boundaryController = new SceneDocumentController(session,
        function(_, _, done) done(null, null), function() {}, function() {
          if (drag != null && drag.commit()) commitCount++;
          drag = null;
        }, function() {
          if (drag != null && drag.cancel()) cancelCount++;
          drag = null;
        });
      drag = PerspectiveSceneDrag.begin(session.scene, camera, "box", 400, 300, 800, 600, false, 0.2);
      check(drag != null, "document-boundary perspective drag begins");
      check(drag.update(camera, 500, 300, 800, 600), "document-boundary drag previews movement");
      var movedX = session.scene.info("box").localTransform().element(12);
      check(!session.scene.document.isDirty, "active drag preview remains outside saved history");
      boundaryController.save();
      check(commitCount == 1 && drag == null, "Save commits the active drag");
      check(!session.scene.document.isDirty &&
        session.scene.document.history.undoCount == historyBeforeDrag + 1,
        "Save establishes a savepoint after the committed move");
      session.scene.document.undo();
      near(session.scene.info("box").localTransform().element(12), -2.0,
        "undo after Save restores the pre-drag position");
      check(session.scene.document.isDirty, "undo before the drag savepoint is dirty");
      session.scene.document.redo();
      near(session.scene.info("box").localTransform().element(12), movedX,
        "redo after Save restores the persisted drag");
      check(!session.scene.document.isDirty, "redo returns to the drag savepoint");

      edit(session, -2.5);
      camera.frame(-2.5, 0.0, 0.0, 1.6, 1.2, 0.05, 4.0 / 3.0);
      function beginDirtyDrag():Void {
        drag = PerspectiveSceneDrag.begin(session.scene, camera, "box", 400, 300, 800, 600, false, 0.2);
        check(drag != null && drag.update(camera, 500, 300, 800, 600),
          "dirty perspective drag previews movement");
      }
      beginDirtyDrag();
      boundaryController.requestNew();
      check(cancelCount == 1 && drag == null && boundaryController.needsConfirmation(),
        "New cancels drag before starting the unsaved-change flow");
      near(session.scene.info("box").localTransform().element(12), -2.5,
        "New interruption restores the pre-drag position");
      boundaryController.resolve("cancel");

      beginDirtyDrag();
      boundaryController.requestOpen();
      check(cancelCount == 2 && boundaryController.needsConfirmation(),
        "Open cancels drag before starting the unsaved-change flow");
      boundaryController.resolve("cancel");
      var boundaryClosed = false;
      beginDirtyDrag();
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
      FileSystem.deleteFile(codeOwned);
      FileSystem.deleteDirectory(directory);
      Sys.println("Scene document tests passed");
    } catch (error:Dynamic) {
      session.dispose();
      throw error;
    }
  }
}

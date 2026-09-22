package tests;

import app.EditorScene;
import app.EditorSceneTree;
import app.EditorSceneViewport;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyValue;
import nativekit.ui.core.PropertyEditResult;
import nativekit.ui.core.ViewportCamera;
import app.SceneDocumentSession;
import sys.FileSystem;
import sys.io.File;

class SceneEditingTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.00001, message);

  static function editingLifecycle():Void {
    var directory = "build/editing-lifecycle-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var file = directory + "/scene.materia.json";
    File.saveContent(file, '{"format":"materia.scene","version":1,"objects":[]}');
    var session = new SceneDocumentSession();
    session.open(file);
    var empty = session.scene;
    var tree = new EditorSceneTree(empty);
    var viewport = new EditorSceneViewport(empty);
    var camera = new ViewportCamera();
    check(empty.selectedId == "scene" && tree.childCount("scene") == 0,
      "empty scene starts with synchronized root selection");
    check(viewport.pick(camera, EditorSceneViewport.ORIGIN_X, EditorSceneViewport.ORIGIN_Y) == "scene",
      "empty viewport has no stale geometry");

    var revision = empty.revision;
    check(empty.createRectangle(), "rectangle creation succeeds");
    var originalId = empty.selectedId;
    check(originalId == "rectangle-1", "created object receives a stable ID");
    check(tree.childCount("scene") == 1 && tree.childKeyAt("scene", 0) == originalId,
      "hierarchy observes created object");
    check(viewport.revision() > revision && viewport.pick(camera,
      EditorSceneViewport.ORIGIN_X, EditorSceneViewport.ORIGIN_Y) == originalId,
      "viewport observes created geometry");
    check(empty.properties().length == 4, "inspector observes created selection");

    var name = new PropertyBinding(empty.properties()[3], empty.context());
    check(name.apply(PropertyValue.Text("Hidden panel")) == PropertyEditResult.Applied,
      "rename succeeds through inspector binding");
    var renamed = empty.object(originalId);
    check(renamed != null && renamed.label == "Hidden panel", "hierarchy model observes rename");
    var visible = new PropertyBinding(empty.properties()[2], empty.context());
    check(visible.apply(PropertyValue.Bool(false)) == PropertyEditResult.Applied,
      "visibility edit succeeds through inspector binding");
    check(viewport.pick(camera, EditorSceneViewport.ORIGIN_X, EditorSceneViewport.ORIGIN_Y) == "scene",
      "hidden object disappears from viewport picking");

    check(empty.duplicateSelected(), "hidden object duplicates");
    var duplicateId = empty.selectedId;
    var duplicate = empty.object(duplicateId);
    check(duplicateId != originalId && duplicate != null && duplicate.label == "Hidden panel copy",
      "duplicate has a distinct stable ID and copied name");
    check(!empty.info(duplicateId).visible() && tree.childKeyAt("scene", 0) == originalId
      && tree.childKeyAt("scene", 1) == duplicateId, "hidden state and hierarchy order duplicate together");
    check(empty.deleteSelected(), "duplicate deletes");
    check(empty.object(duplicateId) == null && empty.selectedId == originalId,
      "delete removes selection and selects its neighbour");
    empty.document.undo();
    check(empty.object(duplicateId) != null && empty.selectedId == duplicateId,
      "undo restores duplicate identity and selection");
    empty.document.redo();
    check(empty.object(duplicateId) == null && empty.selectedId == originalId,
      "redo deletes the same stable identity");

    try {
      session.save(file);
      check(!empty.document.isDirty && session.label().indexOf("*") < 0,
        "save marks the action result clean");
      empty.document.undo();
      check(empty.object(duplicateId) != null && empty.document.isDirty,
        "undo across savepoint restores object and dirty state");
      empty.document.redo();
      check(empty.object(duplicateId) == null && !empty.document.isDirty,
        "redo returns exactly to saved state");
      session.open(file);
      check(session.scene.items().length == 1 && session.scene.items()[0].id == originalId,
        "reopen preserves surviving stable identity");
      check(session.scene.items()[0].label == "Hidden panel" && !session.scene.info(originalId).visible(),
        "reopen preserves rename and hidden state");
      check(!session.scene.document.isDirty && !session.scene.document.canUndo,
        "reopened document starts clean with fresh history");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(file)) FileSystem.deleteFile(file);
      if (FileSystem.exists(directory)) FileSystem.deleteDirectory(directory);
      throw error;
    }
    session.dispose();
    FileSystem.deleteFile(file);
    FileSystem.deleteDirectory(directory);
  }

  static function main():Int {
    var scene = new EditorScene();
    try {
      check(scene.items().length == 2, "two initial objects");
      check(scene.pick(-1.5, 0) == "box", "pick first object");
      check(scene.pick(1.1, 0) == "tower", "pick second object");
      check(scene.pick(8, 8) == "scene", "empty space clears selection");
      var tree = new EditorSceneTree(scene);
      check(tree.childCount("scene") == 2, "hierarchy reflects scene");
      check(tree.childKeyAt("scene", 1) == "tower", "stable hierarchy identity");

      var camera = new ViewportCamera(1.7, 35, -20);
      var viewport = new EditorSceneViewport(scene);
      var screen = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 150,
        EditorSceneViewport.ORIGIN_Y);
      check(viewport.pick(camera, screen.x, screen.y) == "box", "picking after pan and zoom");
      var originalPan = camera.panX;
      var originalRevision = tree.revision();
      var x = new PropertyBinding(scene.properties()[0], scene.context());
      check(x.apply(PropertyValue.Float(-2.5)) == PropertyEditResult.Applied, "position edit accepted");
      near(scene.info("box").localTransform().element(12), -2.5, "selected object moved");
      near(scene.info("tower").localTransform().element(12), 1.1, "other object unchanged");
      near(camera.panX, originalPan, "editing does not move camera");
      check(scene.pick(-2.5, 0) == "box", "picking follows edited geometry");
      check(scene.pick(-1.5, 0) == "scene", "old location no longer hit");
      check(tree.revision() > originalRevision, "scene revision invalidates panels");
      check(scene.document.isDirty && scene.document.canUndo, "edit recorded in document");

      scene.select("tower");
      var towerX = new PropertyBinding(scene.properties()[0], scene.context());
      towerX.apply(PropertyValue.Float(2.0));
      check(scene.document.history.undoCount == 2, "different objects do not coalesce edits");
      scene.document.undo();
      near(scene.info("tower").localTransform().element(12), 1.1, "undo tower edit");
      scene.document.undo();
      near(scene.info("box").localTransform().element(12), -1.5, "undo targets original selection");
      check(!scene.document.isDirty, "undo restores initial document state");
      scene.document.redo();
      near(scene.info("box").localTransform().element(12), -2.5, "redo original object");

      var visibility = new PropertyBinding(scene.properties()[2], scene.context());
      visibility.apply(PropertyValue.Bool(false));
      check(!scene.info("tower").visible(), "hide selected object");
      check(scene.pick(1.1, 0) == "scene", "hidden object cannot be picked");
      check(!scene.document.canRedo, "new edit clears redo branch");
      scene.select("box");
      scene.document.undo();
      check(scene.info("tower").visible(), "undo visibility after selection changes");
      check(scene.pick(1.1, 0) == "tower", "restored object can be picked");

      viewport.frameSelected(camera);
      var center = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 250,
        EditorSceneViewport.ORIGIN_Y);
      near(center.x, viewport.viewportWidth / 2, "frame selection centers X");
      near(center.y, viewport.viewportHeight / 2, "frame selection centers Y");
      scene.select("scene");
      check(scene.properties().length == 0 && !scene.context().hasSelection, "empty selection has no editable properties");
      scene.dispose();
      scene.dispose();
      editingLifecycle();
      SceneDocumentTests.run();
      Sys.println("Scene editing tests passed");
      return 0;
    } catch (error:Dynamic) {
      scene.dispose();
      Sys.println("Scene editing tests failed: " + Std.string(error));
      return 1;
    }
  }
}

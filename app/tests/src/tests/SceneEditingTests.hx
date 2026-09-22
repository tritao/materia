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
    check(empty.properties().length == 7, "inspector observes all created-object properties");

    var name = new PropertyBinding(empty.properties()[3], empty.context());
    check(name.apply(PropertyValue.Text("Hidden panel")) == PropertyEditResult.Applied,
      "rename succeeds through inspector binding");
    var renamed = empty.object(originalId);
    check(renamed != null && renamed.label == "Hidden panel", "hierarchy model observes rename");
    check(new PropertyBinding(empty.properties()[4], empty.context()).apply(PropertyValue.Float(2.4))
      == PropertyEditResult.Applied, "width edit succeeds");
    check(new PropertyBinding(empty.properties()[5], empty.context()).apply(PropertyValue.Float(0.8))
      == PropertyEditResult.Applied, "height edit succeeds");
    check(new PropertyBinding(empty.properties()[6], empty.context()).apply(PropertyValue.Text("#336699"))
      == PropertyEditResult.Applied, "colour edit succeeds");
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
    var duplicateState = empty.items()[1];
    check(!empty.info(duplicateId).visible() && tree.childKeyAt("scene", 0) == originalId
      && tree.childKeyAt("scene", 1) == duplicateId, "hidden state and hierarchy order duplicate together");
    check(duplicateState.width == 2.4 && duplicateState.height == 0.8 && nearValue(duplicateState.red, 0.2),
      "dimensions and colour duplicate together");
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
      var reopened = session.scene.items()[0];
      check(reopened.width == 2.4 && reopened.height == 0.8 && nearValue(reopened.blue, 0.6),
        "reopen preserves dimensions and colour");
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

  static inline function nearValue(actual:Float, expected:Float):Bool
    return Math.abs(actual - expected) < 0.00001;

  static function rectangleProperties():Void {
    var scene = new EditorScene();
    var viewport = new EditorSceneViewport(scene);
    var camera = new ViewportCamera();
    try {
      scene.select("box");
      var width = new PropertyBinding(scene.properties()[4], scene.context());
      var height = new PropertyBinding(scene.properties()[5], scene.context());
      var colour = new PropertyBinding(scene.properties()[6], scene.context());
      check(switch (width.apply(PropertyValue.Float(0.0))) {
        case PropertyEditResult.Rejected(_): true;
        default: false;
      }, "zero width is rejected");
      var notFinite = 0.0;
      notFinite = notFinite / notFinite;
      check(switch (height.apply(PropertyValue.Float(notFinite))) {
        case PropertyEditResult.Rejected(_): true;
        default: false;
      }, "non-finite height is rejected");
      check(switch (colour.apply(PropertyValue.Text("#12GG00"))) {
        case PropertyEditResult.Rejected(_): true;
        default: false;
      }, "invalid colour is rejected");
      check(width.apply(PropertyValue.Float(4.0)) == PropertyEditResult.Applied, "width updates");
      check(height.apply(PropertyValue.Float(0.5)) == PropertyEditResult.Applied, "height updates");
      check(colour.apply(PropertyValue.Text("#33CC66")) == PropertyEditResult.Applied, "colour updates");
      check(scene.pick(0.3, 0.0) == "box", "expanded geometry updates picking bounds");
      check(scene.pick(-1.5, 0.4) == "scene", "reduced geometry removes stale picking bounds");
      viewport.frameSelected(camera);
      near(camera.zoom, Math.min((viewport.viewportWidth - 96.0) / (4.0 * EditorSceneViewport.SCALE),
        (viewport.viewportHeight - 96.0) / (0.5 * EditorSceneViewport.SCALE)),
        "framing uses edited dimensions");
      var item = scene.items()[0];
      check(nearValue(item.red, 0.2) && nearValue(item.green, 0.8)
        && nearValue(item.blue, 0.4), "edited colour updates scene material state");
      scene.document.undo();
      item = scene.items()[0];
      check(nearValue(item.red, 0.22) && nearValue(item.green, 0.52)
        && nearValue(item.blue, 0.85), "colour undo restores exact source channels");
      scene.document.undo();
      item = scene.items()[0];
      check(item.height == 1.2, "height undo restores geometry");
      scene.document.undo();
      item = scene.items()[0];
      check(item.width == 1.6 && scene.pick(0.3, 0.0) == "scene",
        "width undo restores geometry and picking");
      scene.document.redo();
      scene.document.redo();
      scene.document.redo();
      item = scene.items()[0];
      check(item.width == 4.0 && item.height == 0.5 && nearValue(item.green, 0.8),
        "redo restores dimensions and colour");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function viewportDragging():Void {
    var scene = new EditorScene();
    var viewport = new EditorSceneViewport(scene);
    var camera = new ViewportCamera(2.0, 40.0, -30.0);
    try {
      var grabWorld = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 1.2 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.2 * EditorSceneViewport.SCALE);
      check(viewport.beginDrag(camera, grabWorld.x, grabWorld.y, false),
        "left drag begins on an object after camera pan and zoom");
      near(scene.info("box").localTransform().element(12), -1.5, "drag start preserves X grab offset");
      near(scene.info("box").localTransform().element(13), 0.0, "drag start preserves Y grab offset");

      var movedPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 0.0 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.7 * EditorSceneViewport.SCALE);
      check(viewport.updateDrag(camera, movedPointer.x, movedPointer.y), "drag preview moves object");
      near(scene.info("box").localTransform().element(12), -0.3, "preview X includes initial grab offset");
      near(scene.info("box").localTransform().element(13), 0.5, "preview Y includes initial grab offset");
      check(!scene.document.isDirty && scene.document.history.undoCount == 0,
        "drag preview does not add history entries");
      check(viewport.commitDrag(), "release commits moved preview");
      check(scene.document.isDirty && scene.document.history.undoCount == 1,
        "completed drag creates exactly one undo step");
      scene.document.undo();
      near(scene.info("box").localTransform().element(12), -1.5, "undo restores pre-drag X");
      near(scene.info("box").localTransform().element(13), 0.0, "undo restores pre-drag Y");
      scene.document.redo();
      near(scene.info("box").localTransform().element(12), -0.3, "redo restores dragged X");
      near(scene.info("box").localTransform().element(13), 0.5, "redo restores dragged Y");

      var center = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 0.3 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.5 * EditorSceneViewport.SCALE);
      check(viewport.beginDrag(camera, center.x, center.y, false), "second drag begins");
      var cancelledPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 1.3 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y + 0.4 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, cancelledPointer.x, cancelledPointer.y);
      check(viewport.cancelDrag(), "Escape cancels active drag");
      near(scene.info("box").localTransform().element(12), -0.3, "cancel restores original X");
      near(scene.info("box").localTransform().element(13), 0.5, "cancel restores original Y");
      check(scene.document.history.undoCount == 1, "cancel does not create an undo step");

      check(viewport.beginDrag(camera, center.x, center.y, true), "snapped drag begins");
      var snappedPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 0.06 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.94 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, snappedPointer.x, snappedPointer.y);
      near(scene.info("box").localTransform().element(12), 0.0, "grid snapping rounds X");
      near(scene.info("box").localTransform().element(13), 1.0, "grid snapping rounds Y");
      viewport.commitDrag();
      check(scene.document.history.undoCount == 2, "snapped drag commits one undo step");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
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
      rectangleProperties();
      viewportDragging();
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

package tests;

import app.EditorScene;
import app.EditorSceneTree;
import app.EditorSceneViewport;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyValue;
import nativekit.ui.core.PropertyEditResult;
import nativekit.ui.core.ViewportCamera;

class SceneEditingTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.00001, message);

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
      Sys.println("Scene editing tests passed");
      return 0;
    } catch (error:Dynamic) {
      scene.dispose();
      Sys.println("Scene editing tests failed: " + Std.string(error));
      return 1;
    }
  }
}

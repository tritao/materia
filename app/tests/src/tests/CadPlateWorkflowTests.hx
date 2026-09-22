package tests;

import app.CadPlateModel;
import app.CadPlateModel.CadPlateParameters;
import app.EditorScene;
import app.SceneDocumentSession;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyEditResult;
import nativekit.ui.core.PropertyValue;
import sys.FileSystem;
import sys.io.File;

/** End-to-end parametric plate edit, history, persistence, and interchange. */
class CadPlateWorkflowTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  static function object(scene:EditorScene, id:String):app.EditorSceneObject return scene.object(id);
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.00001, message);

  static function parameter(scene:EditorScene, name:String):Float {
    var plate = object(scene, scene.selectedId);
    var model = CadPlateModel.decode(plate.cadGraph);
    var values:CadPlateParameters;
    try {
      values = model.parameters();
    } catch (error:Dynamic) { model.close(); throw error; }
    model.close();
    return switch name {
      case CadPlateModel.WIDTH: values.width;
      case CadPlateModel.HEIGHT: values.height;
      case CadPlateModel.THICKNESS: values.thickness;
      case CadPlateModel.HOLE_DIAMETER: values.holeDiameter;
      case CadPlateModel.HOLE_X: values.holeX;
      case CadPlateModel.HOLE_Y: values.holeY;
      default: throw "Unknown CAD parameter";
    };
  }

  static function edit(scene:EditorScene, label:String, value:Float):PropertyEditResult {
    for (descriptor in scene.properties()) if (descriptor.label == label)
      return new PropertyBinding(descriptor, scene.context()).apply(PropertyValue.Float(value));
    throw "Missing CAD inspector property: " + label;
  }

  static function run():Void {
    var session = new SceneDocumentSession();
    var root = Sys.getCwd() + "/../build-cad";
    if (!FileSystem.exists(root)) FileSystem.createDirectory(root);
    var sceneFile = root + "/plate-workflow.scene";
    var stepFile = root + "/plate-workflow.step";
    try {
      var scene = session.scene;
      check(scene.createMountingPlate(), "create CAD plate");
      var id = scene.selectedId;
      var plate = object(scene, id);
      check(plate.kind == "cad-plate", "selected plate has CAD kind");
      check(scene.pick(0.0, 0.0) == "scene", "initial hole is pick-through");
      check(edit(scene, "Width", 0.1) == PropertyEditResult.Applied, "edit width");
      check(edit(scene, "Height", 0.06) == PropertyEditResult.Applied, "edit height");
      check(edit(scene, "Depth", 0.008) == PropertyEditResult.Applied, "edit thickness");
      check(edit(scene, "Hole diameter", 0.01) == PropertyEditResult.Applied, "edit hole diameter");
      check(edit(scene, "Hole X", 0.02) == PropertyEditResult.Applied, "edit hole X");
      check(edit(scene, "Hole Y", 0.01) == PropertyEditResult.Applied, "edit hole Y");
      near(parameter(scene, CadPlateModel.WIDTH), 0.1, "width recomputed");
      near(parameter(scene, CadPlateModel.HOLE_X), 0.02, "hole offset recomputed");
      check(scene.pick(0.02, 0.01) == "scene", "moved hole remains pick-through");
      check(scene.pick(0.0, 0.0) == id, "old hole location becomes material");
      check(scene.pick(0.049, 0.0) == id && scene.pick(0.051, 0.0) == "scene",
        "resized mesh and picking agree at the boundary");

      var current = object(scene, id);
      var graph = current.cadGraph;
      var undoCount = scene.document.history.undoCount;
      check(edit(scene, "Hole diameter", 0.2) != PropertyEditResult.Applied,
        "invalid hole recompute is rejected");
      check(object(scene, id).cadGraph == graph && scene.document.history.undoCount == undoCount,
        "failed recompute keeps last valid mesh and history");
      check(scene.document.undo(), "undo hole Y");
      near(parameter(scene, CadPlateModel.HOLE_Y), 0.0, "undo restores hole Y");
      check(scene.document.redo(), "redo hole Y");
      near(parameter(scene, CadPlateModel.HOLE_Y), 0.01, "redo restores hole Y");

      check(scene.duplicateSelected(), "duplicate CAD plate");
      var copyId = scene.selectedId;
      check(copyId != id && object(scene, copyId).cadGraph == object(scene, id).cadGraph,
        "duplicate preserves editable feature graph");
      session.save(sceneFile);
      check(!session.isDirty(), "save marks plate document clean");
      session.open(sceneFile);
      scene = session.scene;
      check(object(scene, id) != null && object(scene, copyId) != null,
        "save and reopen restore both plates");
      scene.select(id);
      near(parameter(scene, CadPlateModel.WIDTH), 0.1, "reopen restores width parameter");
      near(parameter(scene, CadPlateModel.HOLE_X), 0.02, "reopen restores hole position");
      scene.exportSelectedCad(stepFile);
      check(FileSystem.exists(stepFile) && File.getContent(stepFile).indexOf("ISO-10303-21") >= 0,
        "selected plate exports STEP");
      check(!session.isDirty(), "STEP export does not dirty the document");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(sceneFile)) FileSystem.deleteFile(sceneFile);
      if (FileSystem.exists(stepFile)) FileSystem.deleteFile(stepFile);
      throw error;
    }
    session.dispose();
    if (FileSystem.exists(sceneFile)) FileSystem.deleteFile(sceneFile);
    if (FileSystem.exists(stepFile)) FileSystem.deleteFile(stepFile);
  }

  static function main():Int {
    try {
      run();
      Sys.println("CAD plate workflow tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println("CAD plate workflow tests failed: " + Std.string(error));
      return 1;
    }
  }
}

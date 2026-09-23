package tests;

import CadKit;
import cadkit.Shape;
import app.CadBracketModel;
import app.CadDocumentSession;
import app.CadPlateModel;
import app.CadPlateModel.CadPlateParameters;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.sketch.SketchConstraint;
import app.EditorScene;
import app.PerspectiveCamera;
import app.SceneDocumentSession;
import app.SceneCodec;
import cadkit.parametric.EvaluationCancelled;
import cadkit.parametric.ParametricError;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyDescriptorOptions;
import nativekit.ui.core.PropertyEditResult;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;
import sys.FileSystem;
import sys.io.File;

/** End-to-end parametric plate edit, history, persistence, and interchange. */
class CadPlateWorkflowTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  static function object(scene:EditorScene, id:String):app.EditorSceneObject return scene.object(id);
  static function near(actual:Float, expected:Float, message:String):Void
    nearWithin(actual, expected, 0.00001, message);

  static function nearWithin(actual:Float, expected:Float, tolerance:Float, message:String):Void
    check(Math.abs(actual - expected) < tolerance,
      message + " (expected " + expected + ", got " + actual + ")");

  static function extrusionDepth(feature:ExtrudeFeature):Float {
    if (feature.amount == null)
      throw "test extrusion has no depth parameter";
    return feature.amount.value;
  }

  static function parameter(scene:EditorScene, name:String):Float {
    var values:CadPlateParameters=scene.cadParameters(scene.selectedId);
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

  static function emptyCadPartWorkflow():Void {
    var scene = new EditorScene([]);
    try {
      check(scene.createCadPart(), "editor creates a generic empty CAD part");
      var id = scene.selectedId;
      check(object(scene, id).kind == "cad-part", "empty CAD part has its own persistent object kind");
      var session = scene.cadSession(id);
      check(session.document.featureCount() == 0 && session.document.outputFeatureOrNull() == null,
        "new CAD part starts with no features or selected output");
      check(session.geometry().vertexCount() == 0, "empty CAD part publishes an empty render resource");

      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var restoredId = reopened.selectedId;
        check(object(reopened, restoredId).kind == "cad-part" &&
          reopened.cadSession(restoredId).document.featureCount() == 0 &&
          reopened.cadSession(restoredId).document.outputFeatureOrNull() == null,
          "empty authored CAD documents survive editor save and reopen");
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function sketchCreationWorkflow():Void {
    var scene = new EditorScene([]);
    try {
      check(scene.createCadPart(), "create a CAD part for sketch authoring");
      var id = scene.selectedId;
      check(scene.createSketch(), "start a new sketch draft in an empty part");
      check(scene.hasActiveSketchEdit() && scene.sketchEditSummary().indexOf("empty") >= 0 &&
        scene.cadSession(id).document.featureCount() == 0,
        "an empty sketch remains a transient draft without creating an invalid feature");
      check(!scene.canApplySelectedSketchEdit(), "an empty draft cannot be applied as a profile");
      check(scene.addSketchDraftRectangle(), "add starter geometry to the empty draft");
      check(scene.canApplySelectedSketchEdit() && scene.sketchEditSummary().indexOf("0 degrees of freedom") >= 0,
        "the draft exposes solver state and enables apply after it forms a closed profile");
      check(edit(scene, "Dimension width", 30) == PropertyEditResult.Applied,
        "starter sketch width can be edited before the feature is created");
      check(scene.applySelectedSketchEdit(), "apply the created sketch draft");
      var feature:ConstrainedSketchFeature = cast scene.cadSession(id).document.featureAt(0);
      near(feature.dimension("width").value, 30, "applied sketch width is authored in the document");
      check(object(scene, id).depth >= 0.000001,
        "planar sketch dimensions stay serializable: " + object(scene, id).depth);

      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var restored:ConstrainedSketchFeature = cast reopened.cadSession(id).document.featureAt(0);
        near(restored.dimension("width").value, 30,
          "created sketch dimensions survive editor save and reopen");
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();

      check(scene.treeSelectionKey() == id + ":feature:0" && scene.beginSelectedSketchEdit(),
        "an applied sketch can reopen as a separate transient draft");
      check(edit(scene, "Dimension width", 32) == PropertyEditResult.Applied,
        "an existing sketch dimension can be edited in the draft");
      check(scene.applySelectedSketchEdit(), "apply the existing sketch edit");
      near(feature.dimension("width").value, 32, "applied edit updates the authored dimension");
      check(scene.document.undo(), "undo sketch dimension edit");
      near(feature.dimension("width").value, 30, "undo restores the previous sketch width");
      check(scene.document.redo(), "redo sketch dimension edit");
      near(feature.dimension("width").value, 32, "redo restores the edited sketch width");
      check(scene.document.undo() && scene.document.undo(), "undo the dimension edit and sketch creation");
      check(scene.cadSession(id).document.outputFeatureOrNull() == null,
        "undoing the only sketch restores the empty part output");

      check(scene.createSketch(), "create another sketch after undoing feature creation");
      check(scene.treeSelectionKey() == id && scene.hasActiveSketchEdit() &&
        scene.addSketchDraftRectangle() && scene.applySelectedSketchEdit(),
        "a new draft can be applied after an earlier feature was undone");
      check(scene.treeSelectionKey() == id + ":feature:1",
        "tree selection retains stable document indexes across inactive undone nodes");
      check(scene.treeSelectionKey() == id + ":feature:1" && scene.beginSelectedSketchEdit(),
        "the second sketch can be reopened for a draft");
      check(scene.clearSketchDraft() && !scene.canApplySelectedSketchEdit(),
        "clearing sketch geometry returns to an unappliable empty draft");
      scene.cancelSelectedSketchEdit();
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function sketchExtrusionWorkflow():Void {
    var scene = new EditorScene([]);
    try {
      check(scene.createCadPart(), "create a CAD part for extrusion authoring");
      var id = scene.selectedId;
      check(scene.createSketch(), "create a sketch to extrude");
      check(scene.addSketchDraftRectangle(), "draw a rectangular extrusion profile");
      check(edit(scene, "Dimension width", 30) == PropertyEditResult.Applied,
        "edit the sketch before extrusion");
      check(scene.applySelectedSketchEdit(), "apply the sketch before extrusion");
      check(scene.canCreateExtrusion(), "a selected solved sketch offers an extrusion action");
      check(scene.createExtrusion(), "extrude the selected sketch into a solid");
      var extrusion:ExtrudeFeature = cast scene.cadSession(id).document.featureAt(1);
      nearWithin(scene.cadSession(id).document.result().volume(), 6000, 0.001,
        "default extrusion uses the authored sketch dimensions");
      check(edit(scene, "Extrusion depth", 12) == PropertyEditResult.Applied,
        "extrusion depth can be edited from its feature inspector");
      near(extrusionDepth(extrusion), 12, "extrusion depth edit updates the authored feature");
      nearWithin(scene.cadSession(id).document.result().volume(), 7200, 0.001,
        "editing extrusion depth recomputes the solid");

      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        check(reopened.cadSession(id).document.featureCount() == 2,
          "sketch and extrusion survive editor save and reopen");
        nearWithin(reopened.cadSession(id).document.result().volume(), 7200, 0.001,
          "reopened extrusion has the same expected solid volume");
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();

      check(scene.document.undo(), "undo extrusion depth edit");
      near(extrusionDepth(extrusion), 10, "undo restores the default extrusion depth");
      check(scene.document.redo(), "redo extrusion depth edit");
      near(extrusionDepth(extrusion), 12, "redo restores the edited extrusion depth");
      check(scene.document.undo() && scene.document.undo(), "undo depth edit then extrusion creation");
      check(!extrusion.active && scene.cadSession(id).document.outputFeatureOrNull() ==
        scene.cadSession(id).document.featureAt(0),
        "undoing extrusion restores the sketch as the active output");
      check(scene.cadFeatureNameAt(id, 1) == "Inactive · extrude",
        "feature tree keeps the stable index of an undone extrusion");
      check(scene.document.redo() && scene.document.redo(), "redo extrusion then its depth edit");
      near(extrusionDepth(extrusion), 12, "redo restores the complete extrusion edit sequence");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function faceSketchPocketWorkflow():Void {
    var scene = new EditorScene([]);
    var step = "create part";
    try {
      check(scene.createCadPart(), "create an editable part for a face-supported pocket");
      var id = scene.selectedId;
      step = "create base sketch";
      check(scene.createSketch(), "create the base profile");
      check(scene.addSketchDraftRectangle(), "draw the base profile");
      check(scene.applySelectedSketchEdit(), "apply the base profile");
      step = "create base extrusion";
      check(scene.createExtrusion(), "create the base solid");
      nearWithin(scene.cadSession(id).document.result().volume(), 4000, 0.001,
        "base extrusion has the expected volume");

      step = "select support face";
      check(scene.selectAtRay(0, 0, 1, 0, 0, -1) == id,
        "ray selection identifies a face on the new solid");
      check(scene.canCreateFaceSketch(), "a selected solid face offers a face sketch");
      step = "create attached sketch";
      check(scene.createFaceSketch() && scene.hasActiveSketchEdit(),
        "face sketch starts in the constrained sketch editor");
      check(!scene.canCreatePocket(), "an unfinished face sketch cannot create a pocket");
      step = "apply attached sketch";
      check(scene.applySelectedSketchEdit(), "apply the face sketch while retaining the solid output");
      check(scene.cadSession(id).document.outputFeatureOrNull() == scene.cadSession(id).document.featureAt(1),
        "face sketch remains an input while the base solid stays visible");
      step = "create pocket";
      check(scene.canCreatePocket() && scene.createPocket(),
        "a solved face sketch creates a through pocket");
      nearWithin(scene.cadSession(id).document.result().volume(), 3640, 0.001,
        "through pocket removes the authored sketch area through the solid");

      step = "edit upstream sketch";
      check(scene.selectTreeKey(id + ":feature:0") && scene.beginSelectedSketchEdit(),
        "the upstream base sketch remains editable after pocket creation");
      step = "change upstream dimension";
      check(edit(scene, "Dimension width", 24) == PropertyEditResult.Applied,
        "upstream sketch dimensions can change below a face-supported pocket");
      step = "recompute downstream pocket";
      check(scene.applySelectedSketchEdit(), "publish the upstream dimension change");
      nearWithin(scene.cadSession(id).document.result().volume(), 4440, 0.001,
        "the selected support face follows its extrusion when the base width changes");
      check(scene.document.undo(), "undo the upstream sketch edit");
      nearWithin(scene.cadSession(id).document.result().volume(), 3640, 0.001,
        "undo restores the previous pocket result");
      check(scene.document.redo(), "redo the upstream sketch edit");
      nearWithin(scene.cadSession(id).document.result().volume(), 4440, 0.001,
        "redo recomputes the face-supported pocket");

      step = "save and reopen";
      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var document = reopened.cadSession(id).document;
        check(document.featureCount() == 4, "base, face sketch, and pocket survive save and reopen");
        nearWithin(document.result().volume(), 4440, 0.001,
          "saved face reference rebuilds the same pocket after reopen");
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      var cause:Dynamic = Reflect.field(error, "cause");
      var inner:Dynamic = cause == null ? null : Reflect.field(cause, "cause");
      var message:Dynamic = inner == null ? null : Reflect.field(inner, "message");
      if (message == null && cause != null)
        message = Reflect.field(cause, "message");
      throw "face sketch pocket workflow failed at " + step + ": " +
        (message == null ? Std.string(error) : Std.string(message));
    }
    scene.dispose();
  }

  static function verticalFilletWorkflow():Void {
    var scene = new EditorScene([]);
    try {
      check(scene.createCadPart(), "create an editable part for edge finishing");
      var id = scene.selectedId;
      check(scene.createSketch(), "create the base profile for edge finishing");
      check(scene.addSketchDraftRectangle(), "draw the edge-finishing profile");
      check(scene.applySelectedSketchEdit(), "apply the edge-finishing profile");
      check(scene.createExtrusion(), "create the solid to fillet");
      var unfilletedVolume = scene.cadSession(id).document.result().volume();
      check(scene.canCreateVerticalFillet() && scene.createVerticalFillet(),
        "the editor creates an explicit vertical-edge fillet query");
      var fillet:cadkit.parametric.features.FilletFeature = cast scene.cadSession(id).document.featureAt(2);
      var defaultVolume = scene.cadSession(id).document.result().volume();
      check(defaultVolume > 0 && defaultVolume < unfilletedVolume,
        "filleting the outside vertical edges preserves a solid and removes corner material");

      check(edit(scene, "Fillet radius", 1) == PropertyEditResult.Applied,
        "fillet radius can be edited from the feature inspector");
      near(fillet.radius.value, 1, "fillet radius edit updates the authored feature");
      var editedVolume = scene.cadSession(id).document.result().volume();
      check(editedVolume > 0 && editedVolume < defaultVolume,
        "a larger fillet recomputes the part");
      check(scene.document.undo(), "undo fillet radius edit");
      near(fillet.radius.value, 0.5, "undo restores the starter fillet radius");
      check(scene.document.redo(), "redo fillet radius edit");
      near(fillet.radius.value, 1, "redo restores the edited fillet radius");

      check(scene.selectTreeKey(id + ":feature:0") && scene.beginSelectedSketchEdit(),
        "the sketch remains editable below the fillet");
      check(edit(scene, "Dimension width", 24) == PropertyEditResult.Applied,
        "upstream profile dimensions can change below the fillet");
      check(scene.applySelectedSketchEdit(), "recompute the fillet after the upstream edit");
      var resizedVolume = scene.cadSession(id).document.result().volume();
      check(resizedVolume > editedVolume,
        "the vertical-edge query follows the resized extrusion");

      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var document = reopened.cadSession(id).document;
        check(document.featureCount() == 3, "sketch, extrusion, and fillet survive save and reopen");
        var restored:cadkit.parametric.features.FilletFeature = cast document.featureAt(2);
        near(restored.radius.value, 1, "fillet radius survives save and reopen");
        nearWithin(document.result().volume(), resizedVolume, 0.001,
          "the edge query restores the same finished solid after reopen");
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function stepImportWorkflow():Void {
    var session = new SceneDocumentSession();
    var root = Sys.getCwd() + "/../build-cad";
    if (!FileSystem.exists(root)) FileSystem.createDirectory(root);
    var sourceFile = root + "/generic-import.step";
    var sceneFile = root + "/generic-import.scene";
    var exportFile = root + "/generic-import-roundtrip.step";
    var source = Shape.box(20, 40, 60);
    try source.exportStep(sourceFile) catch (error:Dynamic) { source.close(); throw error; }
    source.close();
    try {
      var scene = session.scene;
      check(scene.importStep(sourceFile), "import generic STEP body");
      var id = scene.selectedId;
      var importedObject:app.EditorSceneObject = cast scene.object(id);
      check(importedObject.kind == "cad-step", "imported body has a persistent generic CAD kind");
      var cadSession = scene.cadSession(id);
      var imported = cadSession.copyPublishedShape();
      near(imported.volume(), 48000, "generic STEP body has the expected volume");
      imported.close();
      var collision:app.CadCollisionBounds = cast cadSession.collisionBounds;
      near(collision.halfExtents.x, 0.01, "imported collision width follows the source shape");
      near(collision.halfExtents.y, 0.02, "imported collision depth follows the source shape");
      near(collision.halfExtents.z, 0.03, "imported collision height follows the source shape");
      session.save(sceneFile);
      session.open(sceneFile);
      scene = session.scene;
      var reopenedObject:app.EditorSceneObject = cast scene.object(id);
      check(reopenedObject != null && reopenedObject.kind == "cad-step",
        "scene save and reopen retain a generic imported CAD body");
      scene.select(id);
      scene.exportSelectedCad(exportFile);
      check(FileSystem.exists(exportFile) && File.getContent(exportFile).indexOf("ISO-10303-21") >= 0,
        "generic imported body exports STEP after reopening");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(sourceFile)) FileSystem.deleteFile(sourceFile);
      if (FileSystem.exists(sceneFile)) FileSystem.deleteFile(sceneFile);
      if (FileSystem.exists(exportFile)) FileSystem.deleteFile(exportFile);
      throw error;
    }
    session.dispose();
    if (FileSystem.exists(sourceFile)) FileSystem.deleteFile(sourceFile);
    if (FileSystem.exists(sceneFile)) FileSystem.deleteFile(sceneFile);
    if (FileSystem.exists(exportFile)) FileSystem.deleteFile(exportFile);
  }

  static function sketchDraftWorkflow():Void {
    var transientOptions = new PropertyDescriptorOptions();
    transientOptions.recordHistory = false;
    var transientDescriptor = new PropertyDescriptor("test.transient", "Transient", PropertyType.Float,
      function(_) return PropertyValue.Float(0), function(_, _) {}, transientOptions);
    check(!transientDescriptor.recordHistory, "property options preserve transient history policy");
    var model = CadBracketModel.create(0.06, 0.04, 0.03);
    var session = new CadDocumentSession(model);
    try {
      var feature:ConstrainedSketchFeature = cast session.document.featureAt(0);
      var draft = session.beginSketchEdit(0);
      var originalRevision = session.revision;
      check(draft.sketch.isSolved && draft.sketch.degreesOfFreedom == 0,
        "editor opens an isolated draft of a constrained feature");
      check(draft.edit(function(sketch) {
        sketch.replaceConstraint(SketchConstraint.distance("width", "p0", "p1", 61));
      }), "editor draft solves a dimensional change");
      near(feature.dimension("width").value, 60,
        "draft edits do not change the authored feature before apply");
      draft.apply();
      near(feature.dimension("width").value, 61,
        "applying the draft updates its named dimension binding");
      near(model.sceneDimensions().width, 0.061,
        "applying a sketch draft recomputes and publishes the new body");
      check(session.revision == originalRevision + 1 && !draft.active,
        "applying a sketch draft publishes one revision and closes the draft");

      var cancelled = session.beginSketchEdit(0);
      check(cancelled.edit(function(sketch) {
        sketch.replaceConstraint(SketchConstraint.distance("width", "p0", "p1", 62));
      }), "second draft remains independently solvable");
      cancelled.cancel();
      near(feature.dimension("width").value, 61,
        "cancelling a draft leaves the published authored feature unchanged");
      check(session.revision == originalRevision + 1,
        "cancelling a draft does not publish another revision");
    } catch (error:Dynamic) {
      session.close();
      throw error;
    }
    session.close();
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
      var cadSession = scene.cadSession(id);
      var initialCollision:app.CadCollisionBounds = cast cadSession.collisionBounds;
      near(initialCollision.center.x, 0.0, "plate collision proxy is centered on the visual origin");
      near(initialCollision.halfExtents.x, 0.04, "plate collision proxy uses the CAD body's width");
      near(initialCollision.halfExtents.y, 0.025, "plate collision proxy uses the CAD body's height");
      near(initialCollision.halfExtents.z, 0.003, "plate collision proxy uses the CAD body's thickness");
      var sessionRevision = cadSession.revision;
      var retainedShape = cadSession.copyPublishedShape();
      var retainedVolume = retainedShape.volume();
      check(plate.kind == "cad-plate", "selected plate has CAD kind");
      check(scene.pick(0.0, 0.0) == "scene", "initial hole is pick-through");
      check(edit(scene, "Width", 0.1) == PropertyEditResult.Applied, "edit width");
      check(scene.cadSession(id) == cadSession && cadSession.revision > sessionRevision,
        "parameter editing keeps one live CAD document session and publishes a new result revision");
      near(retainedShape.volume(), retainedVolume,
        "a retained published shape stays valid after the session publishes a new revision");
      retainedShape.close();
      check(edit(scene, "Height", 0.06) == PropertyEditResult.Applied, "edit height");
      check(edit(scene, "Depth", 0.008) == PropertyEditResult.Applied, "edit thickness");
      check(edit(scene, "Hole diameter", 0.01) == PropertyEditResult.Applied, "edit hole diameter");
      check(edit(scene, "Hole X", 0.02) == PropertyEditResult.Applied, "edit hole X");
      check(edit(scene, "Hole Y", 0.01) == PropertyEditResult.Applied, "edit hole Y");
      var editedCollision:app.CadCollisionBounds = cast cadSession.collisionBounds;
      near(editedCollision.halfExtents.x, 0.05, "published collision proxy follows changed CAD width");
      near(editedCollision.halfExtents.y, 0.03, "published collision proxy follows changed CAD height");
      near(editedCollision.halfExtents.z, 0.004, "published collision proxy follows changed CAD thickness");
      near(parameter(scene, CadPlateModel.WIDTH), 0.1, "width recomputed");
      near(parameter(scene, CadPlateModel.HOLE_X), 0.02, "hole offset recomputed");
      check(scene.pick(0.02, 0.01) == "scene", "moved hole remains pick-through");
      check(scene.pick(0.0, 0.0) == id, "old hole location becomes material");
      check(scene.pick(0.049, 0.0) == id && scene.pick(0.051, 0.0) == "scene",
        "resized mesh and picking agree at the boundary");

      var current = object(scene, id);
      var graph = scene.currentCadGraph(id);
      var undoCount = scene.document.history.undoCount;
      check(edit(scene, "Hole diameter", 0.2) != PropertyEditResult.Applied,
        "invalid hole recompute is rejected");
      check(scene.currentCadGraph(id) == graph && scene.document.history.undoCount == undoCount,
        "failed recompute keeps last valid mesh and history");
      check(scene.document.undo(), "undo hole Y");
      near(parameter(scene, CadPlateModel.HOLE_Y), 0.0, "undo restores hole Y");
      check(scene.document.redo(), "redo hole Y");
      near(parameter(scene, CadPlateModel.HOLE_Y), 0.01, "redo restores hole Y");

      check(scene.duplicateSelected(), "duplicate CAD plate");
      var copyId = scene.selectedId;
      check(copyId != id && scene.currentCadGraph(copyId) == scene.currentCadGraph(id),
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

  static function faceHoleWorkflow():Void {
    var session = new SceneDocumentSession();
    var root = Sys.getCwd() + "/../build-cad";
    if (!FileSystem.exists(root)) FileSystem.createDirectory(root);
    var sceneFile = root + "/face-hole-workflow.scene";
    try {
      var scene = session.scene;
      check(scene.createMountingPlate(), "create plate for face operation");
      var id = scene.selectedId;
      var camera = new PerspectiveCamera();
      camera.frame(0.02, 0.0, 0.0, 0.08, 0.05, 0.006, 4.0 / 3.0);
      var point:app.PerspectiveCamera.PerspectiveScreenPoint = camera.project(0.02, 0.0, 0.0, 800, 600);
      check(point != null, "project target point into perspective viewport");
      var ray = camera.screenRay(point.x, point.y, 800, 600);
      check(scene.selectAtRay(ray.originX, ray.originY, ray.originZ,
        ray.directionX, ray.directionY, ray.directionZ) == id,
        "perspective ray selects CAD plate material");
      check(scene.canAddHoleOnSelectedFace(), "ray hit retains a selectable CAD face");
      var hitX = scene.selectedCadFaceX, hitY = scene.selectedCadFaceY;
      check(scene.pick(hitX, hitY) == id, "picked face location initially contains material");
      var originalGraph = scene.currentCadGraph(id);
      var originalFeatures = scene.cadFeatureNames(id);
      var undoCount = scene.document.history.undoCount;
      var rejected = false;
      try scene.addHoleOnSelectedFace(0.2) catch (error:ParametricError) {
        rejected = error.message == "Hole must fit inside the plate";
      }
      check(rejected && scene.currentCadGraph(id) == originalGraph &&
        scene.document.history.undoCount == undoCount && scene.canAddHoleOnSelectedFace(),
        "invalid face hole preserves geometry, selection, and history");
      check(scene.addHoleOnSelectedFace(), "add through hole at selected face point");
      check(scene.document.history.undoCount == undoCount + 1,
        "face operation creates exactly one undo step");
      check(scene.currentCadGraph(id) != originalGraph, "face operation updates feature graph");
      check(scene.canAddHoleOnSelectedFace(), "top face selection remaps after recompute");
      check(scene.pick(hitX, hitY) == "scene", "new hole is pick-through at the picked point");
      check(scene.pick(hitX + 0.01, hitY) == id, "material beside new hole stays pickable");
      check(scene.document.undo(), "undo face operation");
      check(scene.cadFeatureNames(id).length == originalFeatures.length && scene.pick(hitX, hitY) == id,
        "undo restores the active feature tree and geometry");
      check(scene.document.redo(), "redo face operation");
      check(scene.pick(hitX, hitY) == "scene", "redo restores new hole geometry");
      session.save(sceneFile);
      check(!session.isDirty(), "face operation savepoint is clean");
      session.open(sceneFile);
      scene = session.scene;
      check(scene.pick(hitX, hitY) == "scene" && scene.pick(hitX + 0.01, hitY) == id,
        "reopen retains face-created hole and material picking");
      check(!session.isDirty(), "reopen preserves clean savepoint");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(sceneFile)) FileSystem.deleteFile(sceneFile);
      throw error;
    }
    session.dispose();
    if (FileSystem.exists(sceneFile)) FileSystem.deleteFile(sceneFile);
  }

  static function bracketWorkflow():Void {
    var session=new SceneDocumentSession();
    var root=Sys.getCwd()+"/../build-cad";
    if(!FileSystem.exists(root))FileSystem.createDirectory(root);
    var sceneFile=root+"/bracket-workflow.scene";
    try {
      var scene=session.scene;
      check(scene.createBracket(),"create constrained L bracket");
      var id=scene.selectedId;
      var modelSession=scene.cadSession(id);
      var metrics=modelSession.performanceMetrics();
      check(metrics.recomputeAttempts>0&&metrics.evaluatedFeatures>0&&metrics.sketchSolveCount>=2,
        "published bracket metrics include feature and sketch work");
      check(metrics.recomputeSeconds>=0&&metrics.sketchSolveSeconds>=0&&
        metrics.sketchProfileSeconds>=0&&metrics.tessellationSeconds>=0&&
        metrics.geometryConversionSeconds>=0&&metrics.publicationSeconds>=0,
        "bracket metrics separate recompute, solve, tessellation, conversion, and publication time");
      check(metrics.nativeShapeHandles>=0&&metrics.nativeMeshHandles>=0&&
        metrics.nativeOperationHandles>=0,
        "native live-handle diagnostics are available for editing-session memory checks");
      var expected=["constrained-sketch","extrude","constrained-sketch","pocket","fillet"];
      var actual=scene.cadFeatureNames(id);
      check(actual.length==expected.length,"bracket exposes its authored feature tree");
      for(index in 0...expected.length)
        check(actual[index]==expected[index],"bracket feature order includes sketch, extrusion, supported pocket, and fillet");
      var profile:ConstrainedSketchFeature=cast modelSession.document.featureAt(0);
      var solvedProfile=profile.solvedSketch();
      check(solvedProfile!=null&&solvedProfile.diagnostic.degreesOfFreedom==0,
        "bracket profile is fully constrained before solid construction");
      var bracketCollision:app.CadCollisionBounds=cast modelSession.collisionBounds;
      check(bracketCollision.halfExtents.x>0&&bracketCollision.halfExtents.y>0&&
        bracketCollision.halfExtents.z>0,
        "bracket exposes an explicit collision proxy from its published solid bounds");
      var bracketModel:app.CadBracketModel=cast modelSession.model;
      var retained=modelSession.copyPublishedShape();
      var volume=retained.volume();
      check(scene.selectTreeKey(id + ":feature:0") && scene.canBeginSelectedSketchEdit(),
        "feature-tree sketch selection exposes constrained sketch editing");
      check(scene.beginSelectedSketchEdit(), "feature-tree command opens a sketch draft");
      var historyBeforeDraftEdit = scene.document.history.undoCount;
      var draftPropertyFound=false;
      for (descriptor in scene.properties()) if (descriptor.label == "Dimension width") {
        draftPropertyFound=true;
        check(!descriptor.recordHistory,"draft dimension properties are transient");
      }
      check(draftPropertyFound,"draft exposes its dimensional constraint in the inspector");
      check(edit(scene,"Dimension width",61)==PropertyEditResult.Applied,
        "sketch inspector edits a draft constraint value");
      check(scene.document.history.undoCount == historyBeforeDraftEdit,
        "transient sketch draft edits do not enter project undo history: "+historyBeforeDraftEdit+" -> "+
          scene.document.history.undoCount+" ("+scene.document.history.undoLabel()+")");
      near(scene.cadSession(id).document.parameter(CadBracketModel.WIDTH).valueIn("mm"),60,
        "inspector draft leaves the part unchanged before apply");
      var authoredBeforeApply:ConstrainedSketchFeature = cast scene.cadSession(id).document.featureAt(0);
      var rawBeforeApply=0.0;
      for (constraint in authoredBeforeApply.sketch().constraints())
        if (constraint.id == "width") rawBeforeApply = constraint.value;
      near(rawBeforeApply,60,"inspector draft does not edit the authored constraint before apply: "+rawBeforeApply);
      check(scene.applySelectedSketchEdit(), "applying the sketch inspector draft succeeds");
      check(scene.document.history.undoCount == historyBeforeDraftEdit + 1,
        "applying a sketch draft creates exactly one project undo operation");
      bracketModel=cast scene.cadSession(id).model;
      near(bracketModel.document.parameter(CadBracketModel.WIDTH).valueIn("mm"),61,
        "applying the sketch inspector draft updates its bound model parameter");
      check(scene.document.history.undoLabel() == "Edit constrained sketch",
        "applying the sketch draft contributes one project undo operation");
      check(scene.document.undo(), "undo sketch inspector draft");
      bracketModel=cast scene.cadSession(id).model;
      var undoWidth=bracketModel.document.parameter(CadBracketModel.WIDTH).valueIn("mm");
      var restoredProfile:ConstrainedSketchFeature=cast bracketModel.document.featureAt(0);
      var restoredRawWidth=0.0;
      for (constraint in restoredProfile.sketch().constraints())
        if (constraint.id == "width") restoredRawWidth = constraint.value;
      near(undoWidth,60,"project undo restores the prior sketch graph: "+undoWidth+" raw "+restoredRawWidth);
      check(scene.document.redo(), "redo sketch inspector draft");
      bracketModel=cast scene.cadSession(id).model;
      near(bracketModel.document.parameter(CadBracketModel.WIDTH).valueIn("mm"),61,
        "project redo restores the edited sketch graph");
      check(scene.document.undo(), "return sketch inspector test to its baseline");
      bracketModel=cast scene.cadSession(id).model;
      var initialRadius=bracketModel.holeRadius();
      check(edit(scene,"Hole radius",0.002)==PropertyEditResult.Applied,
        "inspector edits the bracket's supported hole sketch");
      near(bracketModel.holeRadius(),0.002,"hole sketch radius edits the pocket feature");
      check(scene.document.undo(),"undo bracket hole radius edit");
      near(bracketModel.holeRadius(),initialRadius,"undo restores the bracket hole radius");
      check(scene.document.redo(),"redo bracket hole radius edit");
      near(bracketModel.holeRadius(),0.002,"redo restores the edited bracket hole radius");
      check(edit(scene,"Wall thickness",0.007)==PropertyEditResult.Applied,
        "inspector edits the bracket's constrained wall dimension");
      near(bracketModel.wallThickness(),0.007,"wall edit updates both constrained profile dimensions");
      near(bracketModel.holeRadius(),0.00175,"wall edit keeps the supported hole inside the upright");
      check(scene.document.undo(),"undo bracket wall thickness edit");
      near(bracketModel.wallThickness(),0.006,"undo restores bracket wall thickness");
      near(bracketModel.holeRadius(),0.002,"undo restores the previous hole radius with the wall");
      check(scene.document.redo(),"redo bracket wall thickness edit");
      near(bracketModel.wallThickness(),0.007,"redo restores bracket wall thickness");
      scene.setDimensions(id,0.07,0.045,0.035);
      near(modelSession.model.sceneDimensions().width,0.07,"upstream width edit recomputes the bracket");
      check(modelSession.revision>1&&retained.volume()==volume,
        "recompute publishes a new bracket result while retained geometry remains valid");
      var failed=false,revision=modelSession.revision,undoCount=scene.document.history.undoCount;
      try scene.setDimensions(id,0.01,0.045,0.035) catch(_:ParametricError) failed=true;
      check(failed&&modelSession.revision==revision&&scene.document.history.undoCount==undoCount,
        "an invalid bracket edit preserves its published result and project history");
      retained.close();
      check(scene.document.undo(),"undo bracket dimension edit");
      near(modelSession.model.sceneDimensions().width,0.06,"undo restores the authored bracket dimension");
      check(scene.document.redo(),"redo bracket dimension edit");
      near(modelSession.model.sceneDimensions().width,0.07,"redo reapplies the bracket dimension");
      session.save(sceneFile);
      session.open(sceneFile);
      scene=session.scene;
      scene.select(id);
      check(scene.cadFeatureNames(id).length==expected.length,
        "save and reopen preserve the complete bracket feature graph");
      near(scene.cadSession(id).model.sceneDimensions().height,0.045,
        "save and reopen preserve upstream sketch dimensions");
      var reopenedBracket:app.CadBracketModel=cast scene.cadSession(id).model;
      near(reopenedBracket.wallThickness(),0.007,"save and reopen preserve bracket feature parameters");
      near(reopenedBracket.holeRadius(),0.00175,"save and reopen preserve the supported pocket dimension");
    } catch(error:Dynamic) {
      session.dispose();
      if(FileSystem.exists(sceneFile))FileSystem.deleteFile(sceneFile);
      throw error;
    }
    session.dispose();
    if(FileSystem.exists(sceneFile))FileSystem.deleteFile(sceneFile);
  }

  static function latestOnlyPublication():Void {
    var resourcesBefore=CadKit.resourceCountsGetChecked();
    var session=new SceneDocumentSession();
    try {
      var scene=session.scene;
      check(scene.createBracket(),"create bracket for cancellation checks");
      var cadSession=scene.cadSession(scene.selectedId);
      var dimensions=cadSession.model.sceneDimensions();
      var revision=cadSession.revision;
      var published=cadSession.copyPublishedShape();
      var volume=published.volume();

      var olderPreview=cadSession.beginPreview();
      var newerPreview=cadSession.beginPreview();
      check(!cadSession.publishPreview(olderPreview,published),
        "superseded preview tickets cannot publish");
      check(cadSession.publishPreview(newerPreview,published)&&cadSession.previewResult!=null,
        "current preview tickets publish an owned snapshot");
      cadSession.cancelPreview(olderPreview);
      check(cadSession.previewResult!=null,"cancelling an old preview preserves the current preview");
      cadSession.cancelPreview(newerPreview);
      check(cadSession.previewResult==null,"cancelling the current preview releases its snapshot");

      cadSession.document.afterRecompute=function()cadSession.cancelPendingEvaluation();
      var cancelled=false;
      try cadSession.perform(function(active) {
        active.model.setSceneDimensions(dimensions.width+0.005,dimensions.height,dimensions.depth);
      }) catch(error:EvaluationCancelled) cancelled=true;
      cadSession.document.afterRecompute=null;
      var restored=cadSession.model.sceneDimensions();
      check(cancelled,"a superseded recompute is reported as cancellation");
      check(restored.width==dimensions.width&&restored.height==dimensions.height&&
        restored.depth==dimensions.depth&&cadSession.revision==revision,
        "cancellation rolls authored inputs back without publishing a stale revision");
      check(published.volume()==volume&&cadSession.document.evaluationCancellationCheck==null,
        "cancellation preserves the prior shape and restores the document callback");
      published.close();
    } catch(error:Dynamic) {
      session.dispose();
      throw error;
    }
    session.dispose();
    var resourcesAfter=CadKit.resourceCountsGetChecked();
    check(resourcesAfter.get_shapeCount()==resourcesBefore.get_shapeCount()&&
      resourcesAfter.get_meshCount()==resourcesBefore.get_meshCount()&&
      resourcesAfter.get_operationCount()==resourcesBefore.get_operationCount(),
      "closing the edited session releases its native shapes, meshes, and operations");
  }

  static function main():Int {
    try {
      emptyCadPartWorkflow();
      sketchCreationWorkflow();
      sketchExtrusionWorkflow();
      faceSketchPocketWorkflow();
      verticalFilletWorkflow();
      stepImportWorkflow();
      sketchDraftWorkflow();
      bracketWorkflow();
      faceHoleWorkflow();
      latestOnlyPublication();
      run();
      Sys.println("CAD part workflow tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println("CAD plate workflow tests failed: " + Std.string(error));
      return 1;
    }
  }
}

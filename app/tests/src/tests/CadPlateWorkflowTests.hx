package tests;

import CadKit;
import app.CadPlateModel;
import app.CadPlateModel.CadPlateParameters;
import cadkit.parametric.features.ConstrainedSketchFeature;
import app.EditorScene;
import app.PerspectiveCamera;
import app.SceneDocumentSession;
import cadkit.parametric.EvaluationCancelled;
import cadkit.parametric.ParametricError;
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
      check(profile.solvedSketch()!=null&&profile.solvedSketch().diagnostic.degreesOfFreedom==0,
        "bracket profile is fully constrained before solid construction");
      var bracketModel:app.CadBracketModel=cast modelSession.model;
      var retained=modelSession.copyPublishedShape();
      var volume=retained.volume();
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
      run();
      faceHoleWorkflow();
      bracketWorkflow();
      latestOnlyPublication();
      Sys.println("CAD part workflow tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println("CAD plate workflow tests failed: " + Std.string(error));
      return 1;
    }
  }
}

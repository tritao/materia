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
import app.ProjectDocumentSession;
import app.SceneCodec;
import cadkit.parametric.EvaluationCancelled;
import cadkit.parametric.ParametricError;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReference;
import haxeon.ui.properties.PropertyBinding;
import haxeon.ui.properties.PropertyDescriptor;
import haxeon.ui.properties.PropertyDescriptorOptions;
import haxeon.ui.properties.PropertyEditResult;
import haxeon.ui.properties.PropertyType;
import haxeon.ui.properties.PropertyValue;
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

  /**
   * Reference dimensions in the sketch draft (plan C5.2): the height made a reference shows its measured value
   * read-only and frees a degree of freedom; made driving again it keeps that value.
   */
  static function checkReferenceDimension(scene:EditorScene):Void {
    var height = scene.sketchDraftSolution();
    if (height == null) throw "the draft has no solution";
    var measuredHeight = height.y("p2") - height.y("p1");
    check(toggle(scene, "Reference height (measured)", true) == PropertyEditResult.Applied, "the height can become a reference");
    check(scene.sketchEditSummary().indexOf("1 degree") >= 0, 'a reference height frees the rectangle: ${scene.sketchEditSummary()}');
    var row = [for (descriptor in scene.properties()) if (descriptor.label == "Reference height") descriptor];
    check(row.length == 1 && row[0].readOnly, "the reference height is shown read-only");
    var shown = switch (row[0].readValue(scene.context())) {
      case PropertyValue.Float(value): value;
      default: -1.0;
    };
    near(shown, Math.abs(measuredHeight), "and shows the measured height");

    // Soft drag (plan C5.3): with the height measured, dragging the top corner stretches the rectangle.
    var before = scene.sketchDraftSolution();
    if (before == null) throw "the draft has no solution";
    var top = before.point("p2"), bottom = before.point("p1");
    check(scene.sketchDraftPointNear(top[0] + 0.01, top[1], 0.1) == "p2", "a press near a corner finds it");
    check(scene.beginSketchDraftPointDrag("p2"), "the corner can be dragged");
    check(scene.dragSketchDraftPoint(top[0] + 3, top[1] + 4), "the drag solves");
    var dragging = scene.sketchDraftSolution();
    if (dragging == null) throw "the drag has no solution";
    near(dragging.y("p2"), top[1] + 4, "the corner rises with the cursor");
    near(dragging.x("p2"), top[0], "but stays on its vertical edge");
    near(dragging.y("p1"), bottom[1], "the bottom corner stays put");
    scene.endSketchDraftPointDrag(false);
    var restored = scene.sketchDraftSolution();
    if (restored == null) throw "the cancelled drag left no solution";
    near(restored.y("p2"), top[1], "a cancelled drag restores the draft");
    check(scene.beginSketchDraftPointDrag("p2") && scene.dragSketchDraftPoint(top[0], top[1] + 4), "drag again");
    scene.endSketchDraftPointDrag(true);
    var kept = scene.sketchDraftSnapshot();
    if (kept == null) throw "the kept drag left no draft";
    near([for (point in kept.points()) if (point.id == "p2") point.y][0], top[1] + 4, "a finished drag becomes the draft");

    check(toggle(scene, "Reference height (measured)", false) == PropertyEditResult.Applied, "and driving again");
    check(scene.sketchEditSummary().indexOf("0 degrees of freedom") >= 0, 'which fixes the rectangle again: ${scene.sketchEditSummary()}');
  }

  static function toggle(scene:EditorScene, label:String, value:Bool):PropertyEditResult {
    for (descriptor in scene.properties()) if (descriptor.label == label)
      return new PropertyBinding(descriptor, scene.context()).apply(PropertyValue.Bool(value));
    throw "Missing CAD inspector property: " + label;
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
      check(!scene.addSketchDraftRectangleBetween(4, 5, 4, 15),
        "a zero-width viewport rectangle is ignored");
      check(scene.addSketchDraftRectangleBetween(3, 4, -17, 24),
        "draw a rectangle by dragging between arbitrary workplane points");
      var drawn = scene.sketchDraftSnapshot();
      var drawnSolution = scene.sketchDraftSolution();
      check(drawn != null && drawn.points().length == 4 && drawn.entities().length == 4 &&
        drawn.constraints().length == 7 && drawnSolution != null &&
        scene.canApplySelectedSketchEdit(),
        "the drawn rectangle becomes a solved, fully constrained profile");
      near(drawnSolution.x("rect1.p0"), -17, "dragged rectangle preserves its lower-left point");
      near(drawnSolution.y("rect1.p0"), 4, "dragged rectangle preserves its lower-left point height");
      near(drawnSolution.x("rect1.p1") - drawnSolution.x("rect1.p0"), 20,
        "dragged rectangle width follows the pointer span");
      near(drawnSolution.y("rect1.p2") - drawnSolution.y("rect1.p1"), 20,
        "dragged rectangle height follows the pointer span");
      check(scene.clearSketchDraft(), "clear an interactively drawn sketch before starting another profile");
      check(scene.addSketchDraftRectangle(), "add starter geometry to the empty draft");
      check(scene.canApplySelectedSketchEdit() && scene.sketchEditSummary().indexOf("0 degrees of freedom") >= 0,
        "the draft exposes solver state and enables apply after it forms a closed profile");
      check(edit(scene, "Dimension width", 30) == PropertyEditResult.Applied,
        "starter sketch width can be edited before the feature is created");
      checkReferenceDimension(scene);
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
      var pendingEditReload = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var persistedFeature:ConstrainedSketchFeature = cast pendingEditReload.cadSession(id).document.featureAt(0);
        var pendingWidth:Float = -1;
        var pendingDraft = pendingEditReload.sketchDraftSnapshot();
        if (pendingDraft == null) throw "reopened existing sketch draft is missing";
        for (constraint in pendingDraft.constraints())
          if (constraint.id == "width") pendingWidth = constraint.value;
        check(pendingEditReload.hasActiveSketchEdit() &&
          pendingEditReload.treeSelectionKey() == id + ":feature:0" &&
          persistedFeature.dimension("width").value == 30 && pendingWidth == 32,
          "save and reopen retain an unapplied edit separately from the evaluated feature");
      } catch (error:Dynamic) {
        pendingEditReload.dispose();
        throw error;
      }
      pendingEditReload.dispose();
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

  static function sketchDraftPersistenceWorkflow():Void {
    var file = "/tmp/cadkit-sketch-draft-roundtrip.scene";
    if (FileSystem.exists(file)) FileSystem.deleteFile(file);
    var session = new ProjectDocumentSession();
    try {
      var scene = session.scene;
      check(scene.createCadPart(), "create a part for draft persistence");
      var id = scene.selectedId;
      check(scene.createSketch() && session.isDirty(),
        "starting an empty sketch marks the editor document dirty");
      session.save(file);
      check(!session.isDirty(), "saving an empty sketch draft establishes a savepoint");
      session.open(file);
      scene = session.scene;
      check(scene.selectedId == id && scene.hasActiveSketchEdit() &&
        scene.cadSession(id).document.featureCount() == 0 && !scene.canApplySelectedSketchEdit(),
        "an empty, unappliable sketch draft survives save and reopen");

      check(scene.addSketchDraftRectangleBetween(1, 2, 11, 8) && session.isDirty(),
        "editing a restored draft marks the document dirty");
      session.save(file);
      session.open(file);
      scene = session.scene;
      var restoredDraft = scene.sketchDraftSnapshot();
      var restoredSolution = scene.sketchDraftSolution();
      check(scene.hasActiveSketchEdit() && scene.canApplySelectedSketchEdit() &&
        restoredDraft.points().length == 4 && restoredSolution != null &&
        scene.cadSession(id).document.featureCount() == 0,
        "saved draft geometry reopens as an isolated, solved profile");
      near(restoredSolution.x("rect1.p0"), 1, "saved draft retains its workplane position");
      near(restoredSolution.y("rect1.p0"), 2, "saved draft retains its workplane position");

      check(scene.cancelSelectedSketchEdit() && session.isDirty(),
        "cancelling a saved draft marks its removal for persistence");
      session.save(file);
      session.open(file);
      check(!session.scene.hasActiveSketchEdit() && !session.isDirty(),
        "saving a cancelled draft removes its checkpoint on reopen");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(file)) FileSystem.deleteFile(file);
      throw error;
    }
    session.dispose();
    if (FileSystem.exists(file)) FileSystem.deleteFile(file);
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

  static function supportFaceRepairWorkflow():Void {
    var scene = new EditorScene([]);
    var step = "create a face-supported sketch";
    try {
      check(scene.createCadPart(), "create an editable part for support-reference repair");
      var id = scene.selectedId;
      check(scene.createSketch() && scene.addSketchDraftRectangle() && scene.applySelectedSketchEdit(),
        "create the base sketch");
      check(scene.createExtrusion(), "create the source solid");
      check(scene.selectAtRay(0, 0, 1, 0, 0, -1) == id && scene.createFaceSketch(),
        "create a sketch attached to a selected planar face");
      check(scene.applySelectedSketchEdit(), "apply the attached sketch");
      var feature:ConstrainedSketchFeature = cast scene.cadSession(id).document.featureAt(2);
      var support = feature.support;
      var sourceReference:TopologyReference = feature.supportFaceReference;
      if (support == null || sourceReference == null || sourceReference.currentShape() == null)
        throw "attached sketch has no support face to copy";
      // Exercise the genuinely unresolved case: this attached sketch has no
      // geometric fallback recipe, so undo must restore the unresolved identity.
      var repairTarget = new ConstrainedSketchFeature(feature.sketch(), support, null,
        feature.supportXDirection, feature.supportOffset, feature.supportFlipped,
        sourceReference.currentShape());
      scene.cadSession(id).perform(function(owner) {
        var added:ConstrainedSketchFeature = owner.document.add(repairTarget);
        owner.document.recompute();
      });
      check(scene.selectTreeKey(id + ":feature:3"), "select the attached sketch without a fallback recipe");
      var reference:TopologyReference = repairTarget.supportFaceReference;
      if (reference == null || !reference.isResolved())
        throw "attached sketch did not store a resolved face identity";

      // Model a saved reference whose original support face disappeared. The UI must
      // require an explicit replacement pick before it allows the authored repair.
      var broken = TopologyFingerprint.fromData(CadKit.ShapeKind.Face, CadKit.SurfaceKind.Plane,
        CadKit.CurveKind.Unknown, 1000000000.0, 1000000000.0, 1000000000.0,
        0.0, 0.0, 1.0, 1.0);
      reference.restore(null, broken, ReferenceState.Unresolved);
      check(scene.selectedSketchSupportStatus() != null &&
        !scene.canRepairSelectedSketchSupportFace(),
        "broken support identity is explained and cannot be repaired without selecting a face");

      step = "select a replacement face";
      check(scene.selectAtRay(0, 0, 1, 0, 0, -1) == id &&
        scene.treeSelectionKey() == id + ":feature:3" && scene.canRepairSelectedSketchSupportFace(),
        "picking a replacement face keeps the target sketch selected for repair");
      // TN9: the generic list offers the pick for any reference of the right kind, and names it in words.
      var pickIssues = scene.selectedReferenceIssues();
      check(pickIssues.length == 1 && pickIssues[0].pickable && pickIssues[0].kind == "face",
        "the picked face can repair the broken reference: " + Std.string(pickIssues));
      var picked = scene.selectedElementLabel();
      check(picked != null && picked.indexOf("\u203A") > 0, "the picked face reads as words: " + picked);
      step = "repair the support face";
      check(scene.repairSelectedSketchSupportFace() && reference.isResolved(),
        "repair rebinds the sketch to the explicit face selection");
      var repairedFingerprint = reference.fingerprintData();
      check(scene.document.undo() && reference.state == ReferenceState.Unresolved &&
        reference.fingerprintData().x == broken.x,
        "project undo restores the prior unresolved reference identity");
      check(scene.document.redo() && reference.isResolved() &&
        reference.fingerprintData().x == repairedFingerprint.x &&
        reference.fingerprintData().y == repairedFingerprint.y &&
        reference.fingerprintData().z == repairedFingerprint.z,
        "project redo reapplies the selected support identity");

      step = "save and reopen the repaired face reference";
      var reopened = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
      try {
        var restored:ConstrainedSketchFeature = cast reopened.cadSession(id).document.featureAt(3);
        var restoredReference:TopologyReference = restored.supportFaceReference;
        check(restoredReference != null && restoredReference.isResolved(),
          "the repaired face identity survives save and reopen");
        check(restored.workplane() != null, "the reopened face sketch keeps its support workplane");
      } catch (error:Dynamic) {
        reopened.dispose();
        throw error;
      }
      reopened.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      throw "support-face repair workflow failed at " + step + ": " + Std.string(error);
    }
    scene.dispose();
  }

  /**
    A document that loads with a sketch whose support face was split (plans/TOPOLOGICAL_NAMING.md, TN5): the inspector
    lists the two pieces, choosing one repairs the sketch, and project undo/redo restore and reapply the choice.
  */
  static function splitSupportRepairWorkflow():Void {
    var scene = new EditorScene([]);
    var step = "build a plate that a slot will split";
    try {
      check(scene.createCadPart(), "create an editable part for split-face repair");
      var id = scene.selectedId;
      var session = scene.cadSession(id);
      var slot:Null<cadkit.parametric.features.TransformFeature> = null;
      var sketchIndex = -1;
      session.perform(function(owner) {
        var d = owner.document;
        var plate = d.add(new cadkit.parametric.features.BoxFeature(60, 40, 10));
        var tool = d.add(new cadkit.parametric.features.TransformFeature(d.add(new cadkit.parametric.features.BoxFeature(4, 60, 30)),
          -50, -10, -5));
        slot = tool;
        var body = d.add(new cadkit.parametric.features.BooleanFeature(plate, tool, cadkit.parametric.features.BooleanOperation.Cut));
        d.recompute();
        var shape = body.currentShape();
        var names = shape.elementNames(CadKit.ShapeKind.Face);
        var top = names.indexOf("f" + plate.id.toInt() + ":box.+z");
        var face = shape.subshape(CadKit.ShapeKind.Face, top);
        var sketch = d.add(new ConstrainedSketchFeature(square(), body, null, cadkit.modeling.Vector.X(), 0, false, face));
        face.close();
        d.recompute();
        sketchIndex = d.featureCount() - 1;
      });
      var feature:ConstrainedSketchFeature = cast session.document.featureAt(sketchIndex);
      var reference:TopologyReference = feature.supportFaceReference;
      check(reference.isResolved(), "the sketch sits on the plate's top face");

      step = "load it with the slot across the plate";
      // As a document saved before its support was split would load: the reference stays broken, with its candidates.
      var moving:cadkit.parametric.features.TransformFeature = cast slot;
      session.perform(function(owner) {
        moving.x.set(28);
        try owner.document.recompute() catch (_:Dynamic) {}
      });
      check(reference.state == ReferenceState.Ambiguous, "the split support face is ambiguous");
      check(scene.selectTreeKey(id + ":feature:" + sketchIndex), "select the broken sketch");
      var issues = scene.selectedReferenceIssues();
      check(issues.length == 1 && issues[0].broken && issues[0].candidates.length == 2 &&
        issues[0].message.indexOf("2 elements") >= 0, "the inspector lists the two pieces: " + Std.string(issues));
      // TN9: candidates read as words, the feature tags as the features.
      check(issues[0].candidates[0].indexOf("box ") == 0 && issues[0].candidates[0].indexOf("top (piece)") > 0,
        "candidates name their feature and role: " + issues[0].candidates[0]);

      step = "choose the right-hand piece";
      var candidates = reference.candidates();
      var right = candidates[0].x > candidates[1].x ? 0 : 1;
      check(scene.repairSelectedReference(0, right), "repair the reference with the chosen piece");
      check(reference.isResolved() && reference.currentShape().center().get_x() > 30, "the sketch now sits on the chosen piece");
      check(scene.selectedReferenceIssues().length == 0, "nothing is left to repair");

      step = "undo and redo the repair";
      check(scene.document.undo() && reference.state == ReferenceState.Ambiguous, "project undo restores the broken reference");
      check(scene.document.redo() && reference.isResolved() && reference.currentShape().center().get_x() > 30,
        "project redo reapplies the choice");
    } catch (error:Dynamic) {
      scene.dispose();
      throw "split-face repair workflow failed at " + step + ": " + Std.string(error);
    }
    scene.dispose();
  }

  /**
    An edit that splits the face a sketch sits on (plans/TOPOLOGICAL_NAMING.md, TN7): it stops and asks which piece the
    sketch means; cancelling keeps the previous model, choosing applies the edit with that piece, and undo/redo work.
  */
  static function splitDuringEditWorkflow():Void {
    var scene = new EditorScene([]);
    var step = "build a plate with a slot beside it";
    try {
      check(scene.createCadPart(), "create an editable part for an edit that splits a face");
      var id = scene.selectedId;
      var session = scene.cadSession(id);
      var slotId = -1;
      var sketchIndex = -1;
      session.perform(function(owner) {
        var d = owner.document;
        var plate = d.add(new cadkit.parametric.features.BoxFeature(60, 40, 10));
        var tool = d.add(new cadkit.parametric.features.TransformFeature(d.add(new cadkit.parametric.features.BoxFeature(4, 60, 30)),
          -50, -10, -5));
        slotId = tool.id.toInt();
        var body = d.add(new cadkit.parametric.features.BooleanFeature(plate, tool, cadkit.parametric.features.BooleanOperation.Cut));
        d.recompute();
        var shape = body.currentShape();
        var top = shape.elementNames(CadKit.ShapeKind.Face).indexOf("f" + plate.id.toInt() + ":box.+z");
        var face = shape.subshape(CadKit.ShapeKind.Face, top);
        var sketch:ConstrainedSketchFeature = d.add(new ConstrainedSketchFeature(square(), body, null, cadkit.modeling.Vector.X(), 0, false, face));
        face.close();
        d.recompute();
        sketchIndex = d.featureCount() - 1;
      });
      var feature:ConstrainedSketchFeature = cast session.document.featureAt(sketchIndex);
      var reference:TopologyReference = feature.supportFaceReference;
      check(reference.isResolved() && scene.pendingReferenceChoice() == null, "the sketch sits on the top face");

      step = "move the slot across the plate";
      var failed = false;
      try scene.setCadFeatureParameter(id, slotId, "transform.x", 28) catch (_:Dynamic) failed = true;
      var asked = scene.pendingReferenceChoice();
      check(failed && asked != null, "the edit stops and asks which piece the sketch means");
      var pending:{message:String, candidates:Array<String>} = cast asked;
      check(pending.candidates.length == 2 && pending.message.indexOf("2 elements") >= 0,
        "it offers the two pieces: " + Std.string(pending));

      step = "cancel the edit";
      scene.cancelPendingReferenceChoice();
      var slot = session.document.featureById(slotId);
      check(scene.pendingReferenceChoice() == null && slot.parameter("transform.x").value == -50 && reference.isResolved(),
        "cancelling keeps the previous model");

      step = "make the edit again and choose the right-hand piece";
      try scene.setCadFeatureParameter(id, slotId, "transform.x", 28) catch (_:Dynamic) {}
      check(scene.pendingReferenceChoice() != null, "the edit asks again");
      var choices = reference.candidates();
      check(choices.length == 2, "the reference holds both pieces");
      var right = choices[0].x > choices[1].x ? 0 : 1;
      check(scene.resolvePendingReferenceChoice(right), "apply the edit with the chosen piece");
      check(session.document.featureById(slotId).parameter("transform.x").value == 28 && reference.isResolved() &&
        reference.currentShape().center().get_x() > 30, "the slot moved and the sketch sits on the right-hand piece");

      step = "undo and redo the edit";
      check(scene.document.undo() && session.document.featureById(slotId).parameter("transform.x").value == -50 &&
        reference.isResolved() && reference.currentShape().center().get_x() < 31 && reference.currentShape().center().get_x() > 29,
        "undo puts the slot and the sketch's face back");
      check(scene.document.redo() && reference.isResolved() && reference.currentShape().center().get_x() > 30,
        "redo moves the slot and keeps the chosen piece");
    } catch (error:Dynamic) {
      scene.dispose();
      throw "split-during-edit workflow failed at " + step + ": " + Std.string(error);
    }
    scene.dispose();
  }

  /** A broken fillet edge is repaired by picking an edge in the viewport (plans/TOPOLOGICAL_NAMING.md, TN9). */
  static function edgePickRepairWorkflow():Void {
    var scene = new EditorScene([]);
    var step = "fillet one edge of a box";
    try {
      check(scene.createCadPart(), "create an editable part for edge repair");
      var id = scene.selectedId;
      var session = scene.cadSession(id);
      var filletIndex = -1;
      var boxId = -1;
      session.perform(function(owner) {
        var d = owner.document;
        var box = d.add(new cadkit.parametric.features.BoxFeature(30, 20, 10));
        d.recompute();
        boxId = box.id.toInt();
        var shape = box.currentShape();
        var rim = shape.elementNames(CadKit.ShapeKind.Edge).indexOf('E(f$boxId:box.+x|f$boxId:box.+z)');
        var edge = new cadkit.Edge(shape.subshape(CadKit.ShapeKind.Edge, rim));
        var fillet:cadkit.parametric.features.FilletFeature = d.add(new cadkit.parametric.features.FilletFeature(box, 1, [edge]));
        edge.close();
        d.setOutput(fillet);
        d.recompute();
        filletIndex = d.featureCount() - 1;
      });
      var fillet:cadkit.parametric.features.FilletFeature = cast session.document.featureAt(filletIndex);
      var reference:TopologyReference = fillet.edgeReferences[0];
      check(reference.isResolved(), "the fillet's edge resolves");

      step = "break the edge reference and pick a replacement";
      // As a saved reference whose edge is gone would load.
      var broken = TopologyFingerprint.fromData(CadKit.ShapeKind.Edge, CadKit.SurfaceKind.Unknown, CadKit.CurveKind.Line,
        1e9, 1e9, 1e9, 0, 1, 0, 1, true, "f99:gone");
      reference.restore(null, broken, ReferenceState.Unresolved);
      check(scene.selectTreeKey(id + ":feature:" + filletIndex), "select the fillet");
      var output = session.document.outputFeatureOrNull().currentShape();
      var wanted = 'E(f$boxId:box.+y|f$boxId:box.+z)';
      var pickedEdge = output.elementNames(CadKit.ShapeKind.Edge).indexOf(wanted);
      check(pickedEdge >= 0, "the untouched top/back edge is on the output");
      @:privateAccess scene.selection.selectedCadEdgeIndex = pickedEdge;
      var issues = scene.selectedReferenceIssues();
      check(issues.length == 1 && issues[0].kind == "edge" && issues[0].pickable, "the picked edge can repair it: " + Std.string(issues));
      check(scene.selectedElementLabel().indexOf("box ") == 0 && scene.selectedElementLabel().indexOf("edge between back and top") > 0,
        "the pick reads as words: " + scene.selectedElementLabel());

      step = "repair with the picked edge";
      check(scene.repairSelectedReferenceWithPick(0) && reference.isResolved() && reference.fingerprintData().name == wanted,
        "the fillet now rounds the picked edge");
      check(scene.document.undo() && reference.state == ReferenceState.Unresolved, "undo restores the broken edge");
    } catch (error:Dynamic) {
      scene.dispose();
      throw "edge-pick repair workflow failed at " + step + ": " + Std.string(error);
    }
    scene.dispose();
  }

  static function square():cadkit.sketch.ConstrainedSketch {
    var sketch = new cadkit.sketch.ConstrainedSketch();
    var corners = [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]];
    for (index in 0...4) {
      sketch.addPoint(new cadkit.sketch.SketchPoint("p" + index, corners[index][0], corners[index][1]));
      sketch.addConstraint(SketchConstraint.fixed("fixed" + index, "p" + index));
    }
    for (index in 0...4)
      sketch.addEntity(cadkit.sketch.SketchEntity.line("edge" + index, "p" + index, "p" + ((index + 1) % 4)));
    return sketch;
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
    var session = new ProjectDocumentSession();
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
      var workerTarget = new app.editor.WorkerSceneTargets(scene.records(),scene).box(id);
      if (workerTarget == null) throw "CAD body is unavailable as a worker target";
      near(workerTarget.center[0], importedObject.x + collision.center.x,
        "worker target follows the CAD collision centre");
      near(workerTarget.halfExtents[1], collision.halfExtents.y,
        "worker target follows the CAD collision box");
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
    var session = new ProjectDocumentSession();
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
    var session = new ProjectDocumentSession();
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
    var session=new ProjectDocumentSession();
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
    var session=new ProjectDocumentSession();
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
      sketchDraftPersistenceWorkflow();
      sketchExtrusionWorkflow();
      faceSketchPocketWorkflow();
      supportFaceRepairWorkflow();
      splitSupportRepairWorkflow();
      splitDuringEditWorkflow();
      edgePickRepairWorkflow();
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

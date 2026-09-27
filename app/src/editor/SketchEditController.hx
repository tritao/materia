package app.editor;

import app.CadDocumentSession;
import app.CadSketchEditSession;
import cadkit.modeling.Plane;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SolvedSketch;

/** Active sketch draft state and CAD edit execution. */
class SketchEditController {
  public var activeSketchEdit:Null<CadSketchEditSession> = null;
  public var activeSketchObjectId:Null<String> = null;
  public var sketchEditPlaneValue:Null<Plane> = null;
  public var sketchDraftRevision:Int = 0;
  public var savedSketchDraftRevision:Int = 0;
  public var savedSketchDraftPresent:Bool = false;

  public function new() {}

  public function runCadEdit(session:CadDocumentSession, sync:Void->Void,
      edit:CadDocumentSession->Void, rollback:CadDocumentSession->Void):Void {
    session.perform(edit);
    try sync() catch (error:Dynamic) {
      try { session.perform(rollback); sync(); } catch (_:Dynamic) {}
      throw error;
    }
  }

  public function plane():Null<Plane>
    return activeSketchEdit == null ? null : sketchEditPlaneValue;

  public function snapshot():Null<ConstrainedSketch> {
    var draft = activeSketchEdit;
    return draft == null ? null : draft.sketch.snapshot();
  }

  public function solution():Null<SolvedSketch> {
    if (activeSketchEdit == null) return null;
    var session = activeSketchEdit.sketch;
    return session.solution == null ? session.lastValidSolution : session.solution;
  }

  public function canApply():Bool {
    var draft = activeSketchEdit;
    if (draft == null || !draft.sketch.isSolved) return false;
    try {
      var profile = draft.sketch.buildProfile();
      profile.close();
      return true;
    } catch (_:Dynamic) return false;
  }

  public function summary():Null<String> {
    var draft = activeSketchEdit;
    if (draft == null) return null;
    if (draft.sketch.snapshot().entities().length == 0)
      return "Sketch is empty · drag on the workplane to draw a rectangle";
    var diagnostic = draft.sketch.diagnostic;
    if (diagnostic == null) return "Sketch draft has not been solved";
    return diagnostic.message + " · " + draft.sketch.degreesOfFreedom
      + " degrees of freedom" + (diagnostic.constraintIds.length == 0
        ? "" : " · constraints: " + diagnostic.constraintIds.join(", "))
      + " · drag to add a rectangle";
  }

  public function cancel():Bool {
    if (activeSketchEdit == null) return false;
    activeSketchEdit.cancel();
    activeSketchEdit = null;
    activeSketchObjectId = null;
    sketchEditPlaneValue = null;
    return true;
  }
}

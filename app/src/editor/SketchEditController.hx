package app.editor;

import app.CadDocumentSession;
import app.EditorScene;
import app.CadSketchEditSession;
import cadkit.modeling.Plane;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SolvedSketch;

/** Active sketch draft state and CAD edit execution. */
@:access(app.EditorScene)
class SketchEditController {
  public var activeSketchEdit:Null<CadSketchEditSession> = null;
  public var activeSketchObjectId:Null<String> = null;
  public var sketchEditPlaneValue:Null<Plane> = null;
  public var sketchDraftRevision:Int = 0;
  public var savedSketchDraftRevision:Int = 0;
  public var savedSketchDraftPresent:Bool = false;

  public function new() {}

  public function canAddRectangle():Bool {
    var draft:CadSketchEditSession = cast activeSketchEdit;
    if (draft == null)
      return false;
    var sketch = draft.sketch.snapshot();
    return sketch.points().length == 0 && sketch.entities().length == 0 && sketch.constraints().length == 0;
  }

  /** Add a fully constrained 20 mm starter rectangle to an otherwise empty draft. */
  public function addRectangle(owner:EditorScene):Bool {
    if (!canAddRectangle())
      return false;
    var template = EditorScene.starterSketch();
    owner.editSketchDraft(function(sketch) {
      for (point in template.points()) sketch.addPoint(point);
      for (entity in template.entities()) sketch.addEntity(entity);
      for (constraint in template.constraints()) sketch.addConstraint(constraint);
    });
    return true;
  }

  /** Add a fully constrained rectangle in the current sketch plane. */
  public function addRectangleBetween(owner:EditorScene, startX:Float, startY:Float,
      endX:Float, endY:Float):Bool {
    var draft = activeSketchEdit;
    if (draft == null || !Math.isFinite(startX) || !Math.isFinite(startY) ||
        !Math.isFinite(endX) || !Math.isFinite(endY))
      return false;
    var minX = Math.min(startX, endX), maxX = Math.max(startX, endX);
    var minY = Math.min(startY, endY), maxY = Math.max(startY, endY);
    if (maxX - minX < 0.000001 || maxY - minY < 0.000001 ||
        Math.max(Math.max(Math.abs(minX), Math.abs(maxX)),
          Math.max(Math.abs(minY), Math.abs(maxY))) > 1000000.0)
      return false;
    var sketch = draft.sketch.snapshot();
    var prefixIndex = 1;
    var prefix = "rect" + prefixIndex;
    while (sketchHasPrefix(sketch, prefix)) {
      prefixIndex++;
      prefix = "rect" + prefixIndex;
    }
    var p0 = prefix + ".p0", p1 = prefix + ".p1", p2 = prefix + ".p2", p3 = prefix + ".p3";
    owner.editSketchDraft(function(value) {
      value.addPoint(new SketchPoint(p0, minX, minY));
      value.addPoint(new SketchPoint(p1, maxX, minY));
      value.addPoint(new SketchPoint(p2, maxX, maxY));
      value.addPoint(new SketchPoint(p3, minX, maxY));
      value.addEntity(SketchEntity.line(prefix + ".bottom", p0, p1));
      value.addEntity(SketchEntity.line(prefix + ".right", p1, p2));
      value.addEntity(SketchEntity.line(prefix + ".top", p2, p3));
      value.addEntity(SketchEntity.line(prefix + ".left", p3, p0));
      value.addConstraint(SketchConstraint.fixed(prefix + ".anchor", p0));
      value.addConstraint(SketchConstraint.horizontal(prefix + ".bottom-horizontal", prefix + ".bottom"));
      value.addConstraint(SketchConstraint.vertical(prefix + ".right-vertical", prefix + ".right"));
      value.addConstraint(SketchConstraint.horizontal(prefix + ".top-horizontal", prefix + ".top"));
      value.addConstraint(SketchConstraint.vertical(prefix + ".left-vertical", prefix + ".left"));
      value.addConstraint(SketchConstraint.distance(prefix + ".width", p0, p1, maxX - minX));
      value.addConstraint(SketchConstraint.distance(prefix + ".height", p1, p2, maxY - minY));
    });
    return true;
  }

  function sketchHasPrefix(sketch:ConstrainedSketch, prefix:String):Bool {
    var start = prefix + ".";
    for (point in sketch.points()) if (StringTools.startsWith(point.id, start)) return true;
    for (entity in sketch.entities()) if (StringTools.startsWith(entity.id, start)) return true;
    for (constraint in sketch.constraints()) if (StringTools.startsWith(constraint.id, start)) return true;
    return false;
  }

  public function canClearDraft():Bool {
    var draft = activeSketchEdit;
    if (draft == null)
      return false;
    var sketch = draft.sketch.snapshot();
    return sketch.points().length > 0 || sketch.entities().length > 0 || sketch.constraints().length > 0;
  }

  /** Return a draft to the valid empty state so geometry can be redrawn. */
  public function clearDraft(owner:EditorScene):Bool {
    if (!canClearDraft())
      return false;
    var draft = activeSketchEdit;
    if (draft == null)
      return false;
    var snapshot = draft.sketch.snapshot();
    owner.editSketchDraft(function(sketch) {
      for (constraint in snapshot.constraints()) sketch.removeConstraint(constraint.id);
      for (entity in snapshot.entities()) sketch.removeEntity(entity.id);
      for (point in snapshot.points()) sketch.removePoint(point.id);
    });
    return true;
  }

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

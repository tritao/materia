package app;

import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchSession;

/** Isolated constrained-sketch draft attached to a persistent editor CAD session. */
class CadSketchEditSession {
  public final feature:ConstrainedSketchFeature;
  public final sketch:SketchSession;
  public var active(default, null):Bool = true;

  final owner:CadDocumentSession;

  public function new(owner:CadDocumentSession, feature:ConstrainedSketchFeature) {
    if (owner == null || feature == null)
      throw "CAD sketch edit sessions require an owner and feature";
    if (feature.document != owner.document)
      throw "constrained sketch feature belongs to a different CAD session";
    this.owner = owner;
    this.feature = feature;
    sketch = new SketchSession(feature.sketch());
  }

  /** Solve an isolated edit while leaving the published document untouched. */
  public function edit(change:ConstrainedSketch->Void):Bool {
    ensureActive();
    return sketch.edit(change);
  }

  /** Apply the current solved draft through document recompute and result publication. */
  public function apply():Void {
    ensureActive();
    if (!sketch.isSolved)
      throw "cannot apply a sketch draft with conflicting or invalid constraints";
    owner.perform(function(_) {
      feature.replaceSketch(sketch.snapshot());
      owner.document.recompute();
    });
    active = false;
  }

  /** Discard the isolated draft without changing the authored document. */
  public function cancel():Void {
    ensureActive();
    active = false;
  }

  function ensureActive():Void {
    if (!active)
      throw "CAD sketch edit session is no longer active";
  }
}

package app;

import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchSession;

/** Isolated constrained-sketch draft attached to a persistent editor CAD session. */
class CadSketchEditSession {
  public final feature:Null<ConstrainedSketchFeature>;
  public final sketch:SketchSession;
  public final isNew:Bool;
  public var active(default, null):Bool = true;

  final owner:CadDocumentSession;

  public function new(owner:CadDocumentSession, ?feature:ConstrainedSketchFeature,
      ?initialSketch:ConstrainedSketch) {
    if (owner == null || (feature == null && initialSketch == null))
      throw "CAD sketch edit sessions require an owner and a feature or draft";
    if (feature != null && feature.document != owner.document)
      throw "constrained sketch feature belongs to a different CAD session";
    this.owner = owner;
    this.feature = feature;
    isNew = feature == null;
    var source = initialSketch;
    if (source == null && feature != null)
      source = feature.sketch();
    if (source == null)
      throw "CAD sketch edit sessions require an authored sketch";
    sketch = new SketchSession(source);
  }

  /** Solve an isolated edit while leaving the published document untouched. */
  public function edit(change:ConstrainedSketch->Void):Bool {
    ensureActive();
    return sketch.edit(change);
  }

  /** Apply the current solved draft through document recompute and result publication. */
  public function apply():Void {
    ensureActive();
    if (feature == null)
      throw "new sketch drafts must be applied through the editor document history";
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

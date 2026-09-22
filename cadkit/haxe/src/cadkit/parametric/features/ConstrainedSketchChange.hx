package cadkit.parametric.features;

import cadkit.parametric.DocumentChange;
import cadkit.sketch.ConstrainedSketch;

class ConstrainedSketchChange implements DocumentChange {
	private final feature:ConstrainedSketchFeature;
	private final before:ConstrainedSketch;
	private final after:ConstrainedSketch;

	public function new(feature:ConstrainedSketchFeature, before:ConstrainedSketch, after:ConstrainedSketch) {
		this.feature = feature;
		this.before = before;
		this.after = after;
	}

	public function undo():Void {
		feature.restoreSketch(before);
	}

	public function redo():Void {
		feature.restoreSketch(after);
	}
}

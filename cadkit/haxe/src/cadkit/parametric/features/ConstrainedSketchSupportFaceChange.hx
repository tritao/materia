package cadkit.parametric.features;

import cadkit.parametric.DocumentChange;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyFingerprint;

/** Undoable identity change for an attached sketch's selected support face. */
class ConstrainedSketchSupportFaceChange implements DocumentChange {
	final feature:ConstrainedSketchFeature;
	final beforeFingerprint:TopologyFingerprint;
	final beforeState:ReferenceState;
	final afterFingerprint:TopologyFingerprint;

	public function new(feature:ConstrainedSketchFeature, beforeFingerprint:TopologyFingerprint,
		beforeState:ReferenceState, afterFingerprint:TopologyFingerprint) {
		this.feature = feature;
		this.beforeFingerprint = beforeFingerprint;
		this.beforeState = beforeState;
		this.afterFingerprint = afterFingerprint;
	}

	public function undo():Void
		feature.restoreSupportFaceReference(beforeFingerprint, beforeState);

	public function redo():Void
		feature.restoreSupportFaceReference(afterFingerprint, ReferenceState.Resolved);
}

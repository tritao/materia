package cadkit.parametric;

/**
	Undoable retargeting of a topology reference (plans/TOPOLOGICAL_NAMING.md, TN5): the user chose which element a
	broken reference means. The new identity is pending until the next recompute finds it, by its name first.
*/
class TopologyReferenceChange implements DocumentChange {
	final reference:TopologyReference;
	final beforeFingerprint:TopologyFingerprint;
	final beforeState:ReferenceState;
	final afterFingerprint:TopologyFingerprint;

	public function new(reference:TopologyReference, beforeFingerprint:TopologyFingerprint, beforeState:ReferenceState,
			afterFingerprint:TopologyFingerprint) {
		this.reference = reference;
		this.beforeFingerprint = beforeFingerprint;
		this.beforeState = beforeState;
		this.afterFingerprint = afterFingerprint;
	}

	public function undo():Void
		apply(reference, beforeFingerprint, beforeState);

	public function redo():Void
		apply(reference, afterFingerprint, ReferenceState.Unresolved);

	/**
		Put `reference` back to `fingerprint` in `state`. A resolved state needs its element in the producer's current
		shape; when that is gone the reference is left unresolved, for the next recompute to report.
	*/
	public static function apply(reference:TopologyReference, fingerprint:TopologyFingerprint, state:ReferenceState):Void {
		var topology:Null<cadkit.Shape> = null;
		var target = state;
		if (state == ReferenceState.Resolved || state == ReferenceState.Remapped) {
			var shape = reference.remapTargetFeature().currentShape();
			var resolution = shape == null ? null : TopologyResolver.resolve(shape, fingerprint, reference.kind);
			if (resolution != null && resolution.state == ReferenceState.Resolved)
				topology = shape.subshape(reference.kind, resolution.index);
			else
				target = ReferenceState.Unresolved;
		}
		try {
			reference.restore(topology, fingerprint, target);
		} catch (error:Dynamic) {
			if (topology != null)
				topology.close();
			throw error;
		}
		if (topology != null)
			topology.close();
		reference.feature.markDirty();
	}
}

package cadkit.parametric;

/** Activation of a retained feature graph node participates in undo and redo. */
class FeatureActiveChange implements DocumentChange {
	private final feature:Feature;
	private final before:Bool;
	private final after:Bool;

	public function new(feature:Feature, before:Bool, after:Bool) {
		this.feature = feature;
		this.before = before;
		this.after = after;
	}

	public function undo():Void
		feature.restoreActive(before);

	public function redo():Void
		feature.restoreActive(after);
}

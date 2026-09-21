package cadkit.parametric;

/** Stable identifier for a feature within one document. */
abstract FeatureId(Int) from Int to Int {
	public inline function new(value:Int) {
		this = value;
	}

	public inline function toInt():Int {
		return this;
	}
}

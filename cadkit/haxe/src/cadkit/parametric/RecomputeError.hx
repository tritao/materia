package cadkit.parametric;

import cadkit.parametric.FeatureId;

/** A feature failed while a document was being recomputed. */
class RecomputeError {
	public final feature:FeatureId;
	public final cause:Dynamic;

	public function new(feature:FeatureId, cause:Dynamic) {
		this.feature = feature;
		this.cause = cause;
	}

	public function toString():String {
		return "feature " + feature.toInt() + " failed: " + Std.string(cause);
	}
}

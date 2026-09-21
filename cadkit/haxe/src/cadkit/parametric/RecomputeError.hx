package cadkit.parametric;

import cadkit.parametric.FeatureId;

/** A feature failed while a document was being recomputed. */
class RecomputeError {
	public final feature:FeatureId;
	public final cause:Dynamic;
	public final referenceState:Null<ReferenceState>;

	public function new(feature:FeatureId, cause:Dynamic) {
		this.feature = feature;
		this.cause = cause;
		if (Std.isOfType(cause, ParametricError)) {
			var parametric:ParametricError = cast cause;
			referenceState = parametric.referenceState;
		} else
			referenceState = null;
	}

	public function toString():String {
		var stateSuffix = referenceState == null ? "" : " [" + Std.string(referenceState) + "]";
		return "feature " + feature.toInt() + " failed" + stateSuffix + ": " + Std.string(cause);
	}
}

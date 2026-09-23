package cadkit.parametric;

import CadKit;
import cadkit.Shape;

/** One conservative fallback policy shared by features, references, and editor adapters. */
class TopologyResolver {
	private static inline var InvalidScore:Float = -1.0e30;
	private static inline var AmbiguityMargin:Float = 0.025;

	public static function resolve(shape:Shape, fingerprint:TopologyFingerprint, kind:CadKit.ShapeKind,
		?identity:Shape):TopologyResolution {
		if (fingerprint.kind != kind)
			return new TopologyResolution(-1, ReferenceState.Unresolved);

		var bestIndex = -1;
		var bestScore = InvalidScore;
		var secondBestScore = InvalidScore;
		var count = shape.subshapeCount(kind);
		for (index in 0...count) {
			var candidate = shape.subshape(kind, index);
			if (identity != null && identity.sameAs(candidate)) {
				candidate.close();
				return new TopologyResolution(index, ReferenceState.Resolved);
			}
			var score = fingerprint.score(candidate);
			candidate.close();
			if (score <= InvalidScore / 2.0)
				continue;
			if (score > bestScore) {
				secondBestScore = bestScore;
				bestScore = score;
				bestIndex = index;
			} else if (score > secondBestScore) {
				secondBestScore = score;
			}
		}

		if (bestIndex < 0 || bestScore < 0.5)
			return new TopologyResolution(-1, ReferenceState.Unresolved);
		if (secondBestScore > InvalidScore / 2.0 && bestScore - secondBestScore <= AmbiguityMargin)
			return new TopologyResolution(-1, ReferenceState.Ambiguous);
		return new TopologyResolution(bestIndex, ReferenceState.Resolved);
	}
}

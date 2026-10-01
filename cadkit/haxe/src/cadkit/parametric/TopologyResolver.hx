package cadkit.parametric;

import CadKit;
import cadkit.ElementNames;
import cadkit.Shape;

/**
	One conservative resolution policy shared by features, references, and editor adapters (plans/TOPOLOGICAL_NAMING.md,
	TN-D9): the element's name first, its geometry as the tie-breaker among names and as the last resort. Ties are
	reported as ambiguous, never guessed.
*/
class TopologyResolver {
	private static inline var InvalidScore:Float = -1.0e30;
	private static inline var AmbiguityMargin:Float = 0.025;
	/** A relative name match must beat the next by this much to win without the geometry. */
	private static inline var RelativeMargin:Float = 0.05;

	public static function resolve(shape:Shape, fingerprint:TopologyFingerprint, kind:CadKit.ShapeKind,
		?identity:Shape):TopologyResolution {
		if (fingerprint.kind != kind)
			return new TopologyResolution(-1, ReferenceState.Unresolved);
		if (fingerprint.name != null) {
			var named = byName(fingerprint.name, shape.elementNames(kind), index -> {
				var candidate = shape.subshape(kind, index);
				var score = fingerprint.score(candidate);
				candidate.close();
				return score;
			});
			if (named != null && named.state != ReferenceState.Ambiguous)
				return named;
			if (named != null) {
				// Names left candidates tied: an exact geometric match still decides, otherwise it is ambiguous.
				var geometric = TopologyResolver.resolve(shape, TopologyFingerprint.withoutName(fingerprint), kind, identity);
				return geometric.state == ReferenceState.Resolved ? geometric : named;
			}
		}

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

		return choose(bestIndex, bestScore, secondBestScore);
	}

	/**
		`resolve` over candidates' fingerprints instead of their shapes (an editor holding face descriptors but
		not the B-rep): the index into `candidates` that matches, under the same scoring and ambiguity policy.
	*/
	public static function resolveAmong(candidates:Array<TopologyFingerprint>, fingerprint:TopologyFingerprint):TopologyResolution {
		if (fingerprint.name != null) {
			var names = [for (candidate in candidates) candidate.name == null ? "" : candidate.name];
			var named = byName(fingerprint.name, names, index -> fingerprint.scoreAgainst(candidates[index]));
			if (named != null && named.state != ReferenceState.Ambiguous)
				return named;
			if (named != null) {
				var geometric = resolveAmong(candidates, TopologyFingerprint.withoutName(fingerprint));
				return geometric.state == ReferenceState.Resolved ? geometric : named;
			}
		}
		var bestIndex = -1;
		var bestScore = InvalidScore;
		var secondBestScore = InvalidScore;
		for (index in 0...candidates.length) {
			var score = fingerprint.scoreAgainst(candidates[index]);
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
		return choose(bestIndex, bestScore, secondBestScore);
	}

	/**
		Resolution by name, or null to fall back to geometry: one exact name resolves; several exact names, weak names
		and tied relatives are decided by `geometry` (a fingerprint score per index) among those candidates only. A weak
		name the geometry does not confirm falls back; several named candidates the geometry cannot separate are ambiguous.
	*/
	static function byName(reference:String, names:Array<String>, geometry:Int->Float):Null<TopologyResolution> {
		var scores = ElementNames.match(reference, names);
		var exact:Array<Int> = [], weak:Array<Int> = [], relative:Array<Int> = [];
		for (index in 0...scores.length) {
			var score = scores[index];
			if (score >= ElementNames.EXACT)
				exact.push(index);
			else if (score >= ElementNames.WEAK)
				weak.push(index);
			else if (score >= ElementNames.RELATIVE)
				relative.push(index);
		}
		if (exact.length == 1)
			return new TopologyResolution(exact[0], ReferenceState.Resolved);
		if (exact.length > 1)
			return amongNamed(exact, geometry, true);
		if (weak.length > 0) {
			var confirmed = amongNamed(weak, geometry, false);
			return confirmed.state == ReferenceState.Resolved ? confirmed : null;
		}
		if (relative.length == 0)
			return null;
		var best = -1.0, second = -1.0;
		for (index in relative) {
			if (scores[index] > best) {
				second = best;
				best = scores[index];
			} else if (scores[index] > second) {
				second = scores[index];
			}
		}
		if (relative.length == 1 || best - second >= RelativeMargin)
			for (index in relative)
				if (scores[index] == best)
					return new TopologyResolution(index, ReferenceState.Resolved);
		return amongNamed([for (index in relative) if (best - scores[index] < RelativeMargin) index], geometry, true);
	}

	/** The one of `indices` the geometry picks; otherwise ambiguous (or unresolved when `ambiguous` is false). */
	static function amongNamed(indices:Array<Int>, geometry:Int->Float, ambiguous:Bool):TopologyResolution {
		var bestIndex = -1;
		var bestScore = InvalidScore;
		var secondBestScore = InvalidScore;
		for (index in indices) {
			var score = geometry(index);
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
		var chosen = choose(bestIndex, bestScore, secondBestScore);
		if (chosen.state == ReferenceState.Resolved || !ambiguous)
			return chosen;
		return new TopologyResolution(-1, ReferenceState.Ambiguous);
	}

	static function choose(bestIndex:Int, bestScore:Float, secondBestScore:Float):TopologyResolution {
		if (bestIndex < 0 || bestScore < 0.5)
			return new TopologyResolution(-1, ReferenceState.Unresolved);
		if (secondBestScore > InvalidScore / 2.0 && bestScore - secondBestScore <= AmbiguityMargin)
			return new TopologyResolution(-1, ReferenceState.Ambiguous);
		return new TopologyResolution(bestIndex, ReferenceState.Resolved);
	}
}

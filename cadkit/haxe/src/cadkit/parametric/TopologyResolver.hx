package cadkit.parametric;

import CadKit;
import cadkit.ElementNames;
import cadkit.Shape;
import cadkit.parametric.TopologyResolution.ResolutionMethod;

/**
	One conservative resolution policy shared by features, references, and editor adapters (plans/TOPOLOGICAL_NAMING.md,
	TN-D9): the element's name first, its geometry as the tie-breaker among names and as the last resort. Ties are
	reported as ambiguous with their candidates, never guessed.
*/
class TopologyResolver {
	private static inline var InvalidScore:Float = -1.0e30;
	private static inline var AmbiguityMargin:Float = 0.025;
	/** A relative name match must beat the next by this much overlap to win without the geometry. */
	private static inline var RelativeMargin:Float = 0.05;

	public static function resolve(shape:Shape, fingerprint:TopologyFingerprint, kind:CadKit.ShapeKind,
		?identity:Shape):TopologyResolution {
		if (fingerprint.kind != kind)
			return new TopologyResolution(-1, ReferenceState.Unresolved);
		var geometry = (index:Int) -> {
			var candidate = shape.subshape(kind, index);
			var score = fingerprint.score(candidate);
			candidate.close();
			return score;
		};
		var named = fingerprint.name == null ? null : byName(fingerprint.name, shape.elementNames(kind), geometry);
		if (named != null && named.state != ReferenceState.Ambiguous)
			return named;

		var count = shape.subshapeCount(kind);
		if (identity != null) {
			for (index in 0...count) {
				var candidate = shape.subshape(kind, index);
				var same = identity.sameAs(candidate);
				candidate.close();
				if (same)
					return new TopologyResolution(index, ReferenceState.Resolved, ResolutionMethod.Identity);
			}
		}
		var geometric = rank([for (index in 0...count) index], geometry);
		// Names left candidates tied: an exact geometric match still decides, otherwise it is ambiguous among them.
		return named == null || geometric.state == ReferenceState.Resolved ? geometric : named;
	}

	/**
		`resolve` over candidates' fingerprints instead of their shapes (an editor holding face descriptors but
		not the B-rep): the index into `candidates` that matches, under the same policy.
	*/
	public static function resolveAmong(candidates:Array<TopologyFingerprint>, fingerprint:TopologyFingerprint):TopologyResolution {
		var geometry = (index:Int) -> fingerprint.scoreAgainst(candidates[index]);
		var named = fingerprint.name == null ? null
			: byName(fingerprint.name, [for (candidate in candidates) candidate.name == null ? "" : candidate.name], geometry);
		if (named != null && named.state != ReferenceState.Ambiguous)
			return named;
		var geometric = rank([for (index in 0...candidates.length) index], geometry);
		return named == null || geometric.state == ReferenceState.Resolved ? geometric : named;
	}

	/**
		Resolution by name, or null to fall back to geometry: one exact name resolves; several exact names, weak names
		and tied relatives are decided by `geometry` among those candidates only. A weak name the geometry does not
		confirm falls back; named candidates the geometry cannot separate are ambiguous.
	*/
	static function byName(reference:String, names:Array<String>, geometry:Int->Float):Null<TopologyResolution> {
		var matches = ElementNames.match(reference, names);
		var exact:Array<Int> = [], weak:Array<Int> = [], relative:Array<Int> = [];
		for (index in 0...matches.length) {
			var grade = matches[index].grade;
			if (grade == ElementNames.EXACT)
				exact.push(index);
			else if (grade == ElementNames.WEAK)
				weak.push(index);
			else if (grade == ElementNames.RELATIVE)
				relative.push(index);
		}
		if (exact.length == 1)
			return new TopologyResolution(exact[0], ReferenceState.Resolved, ResolutionMethod.Name);
		if (exact.length > 1)
			return named(rank(exact, geometry), ResolutionMethod.Name, exact);
		if (weak.length > 0) {
			var confirmed = rank(weak, geometry);
			return confirmed.state == ReferenceState.Resolved ? new TopologyResolution(confirmed.index, ReferenceState.Resolved, ResolutionMethod.Name) : null;
		}
		if (relative.length == 0)
			return null;
		relative.sort((a, b) -> matches[b].overlap > matches[a].overlap ? 1 : matches[b].overlap < matches[a].overlap ? -1 : a - b);
		var best = matches[relative[0]].overlap;
		var tied = [for (index in relative) if (best - matches[index].overlap < RelativeMargin) index];
		if (tied.length == 1)
			return new TopologyResolution(tied[0], ReferenceState.Resolved, ResolutionMethod.Relative);
		return named(rank(tied, geometry), ResolutionMethod.Relative, tied);
	}

	/** `ranked` among named candidates: resolved by `method`, or ambiguous between all of `candidates`. */
	static function named(ranked:TopologyResolution, method:ResolutionMethod, candidates:Array<Int>):TopologyResolution {
		if (ranked.state == ReferenceState.Resolved)
			return new TopologyResolution(ranked.index, ReferenceState.Resolved, method);
		return new TopologyResolution(-1, ReferenceState.Ambiguous, ResolutionMethod.NotFound, candidates);
	}

	/** The one of `indices` the geometry picks, or ambiguous between the near-best, or unresolved. */
	static function rank(indices:Array<Int>, geometry:Int->Float):TopologyResolution {
		var scored:Array<{index:Int, score:Float}> = [];
		for (index in indices) {
			var score = geometry(index);
			if (score > InvalidScore / 2.0)
				scored.push({index: index, score: score});
		}
		scored.sort((a, b) -> b.score > a.score ? 1 : b.score < a.score ? -1 : a.index - b.index);
		if (scored.length == 0 || scored[0].score < 0.5)
			return new TopologyResolution(-1, ReferenceState.Unresolved);
		if (scored.length > 1 && scored[0].score - scored[1].score <= AmbiguityMargin)
			return new TopologyResolution(-1, ReferenceState.Ambiguous, ResolutionMethod.NotFound,
				[for (entry in scored) if (scored[0].score - entry.score <= AmbiguityMargin) entry.index]);
		return new TopologyResolution(scored[0].index, ReferenceState.Resolved, ResolutionMethod.Geometry);
	}
}

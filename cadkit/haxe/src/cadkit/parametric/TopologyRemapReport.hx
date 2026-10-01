package cadkit.parametric;

import cadkit.parametric.TopologyResolution.ResolutionMethod;

/** Aggregate topology-reference outcomes from one document recompute. */
class TopologyRemapReport {
	public var remapped(default, null):Int;
	public var deleted(default, null):Int;
	public var unresolved(default, null):Int;
	public var ambiguous(default, null):Int;
	/** How the found ones were found (plans/TOPOLOGICAL_NAMING.md): by name, as a relative, unchanged, by geometry, by selection. */
	public var byName(default, null):Int;
	public var byRelative(default, null):Int;
	public var byIdentity(default, null):Int;
	public var byGeometry(default, null):Int;
	public var bySelection(default, null):Int;

	public function new() {
		remapped = 0;
		deleted = 0;
		unresolved = 0;
		ambiguous = 0;
		byName = 0;
		byRelative = 0;
		byIdentity = 0;
		byGeometry = 0;
		bySelection = 0;
	}

	public function add(state:ReferenceState, ?method:ResolutionMethod):Void {
		if (method != null)
			switch method {
				case ResolutionMethod.Name: byName++;
				case ResolutionMethod.Relative: byRelative++;
				case ResolutionMethod.Identity: byIdentity++;
				case ResolutionMethod.Geometry: byGeometry++;
				case ResolutionMethod.Selection: bySelection++;
				case ResolutionMethod.NotFound:
			}
		switch state {
			case Remapped:
				remapped++;
			case Deleted:
				deleted++;
			case Unresolved:
				unresolved++;
			case Ambiguous:
				ambiguous++;
			case Resolved, Closed:
			}
	}

	public function merge(other:TopologyRemapReport):Void {
		remapped += other.remapped;
		deleted += other.deleted;
		unresolved += other.unresolved;
		ambiguous += other.ambiguous;
		byName += other.byName;
		byRelative += other.byRelative;
		byIdentity += other.byIdentity;
		byGeometry += other.byGeometry;
		bySelection += other.bySelection;
	}

	public function total():Int {
		return remapped + deleted + unresolved + ambiguous;
	}
}

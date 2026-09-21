package cadkit.parametric;

/** Aggregate topology-reference outcomes from one document recompute. */
class TopologyRemapReport {
	public var remapped(default, null):Int;
	public var deleted(default, null):Int;
	public var unresolved(default, null):Int;
	public var ambiguous(default, null):Int;

	public function new() {
		remapped = 0;
		deleted = 0;
		unresolved = 0;
		ambiguous = 0;
	}

	public function add(state:ReferenceState):Void {
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
	}

	public function total():Int {
		return remapped + deleted + unresolved + ambiguous;
	}
}

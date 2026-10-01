package cadkit.parametric;

/** How a topology reference found its element (plans/TOPOLOGICAL_NAMING.md, TN-D9). */
enum abstract ResolutionMethod(String) to String {
	/** Its stored name, exactly (geometry may have broken a tie between equal names). */
	var Name = "name";
	/** A relative of its name: a piece of the face it was on, or the face its piece became. */
	var Relative = "relative";
	/** The very same topology, unchanged since it was captured. */
	var Identity = "identity";
	/** Its geometry alone: a reference without a name, or whose name is gone. */
	var Geometry = "geometry";
	/** The explicit selection recipe kept as its fallback. */
	var Selection = "selection";
	/** Not found (unresolved, ambiguous or deleted). */
	var NotFound = "none";
}

class TopologyResolution {
	/** The element's index among the shape's subshapes of its kind; -1 unless resolved. */
	public final index:Int;
	public final state:ReferenceState;
	public final method:ResolutionMethod;
	/** When ambiguous: the indices it could not choose between (best first); otherwise empty. */
	public final candidates:Array<Int>;

	public function new(index:Int, state:ReferenceState, ?method:ResolutionMethod, ?candidates:Array<Int>) {
		this.index = index;
		this.state = state;
		this.method = method == null ? (state == ReferenceState.Resolved ? ResolutionMethod.Geometry : ResolutionMethod.NotFound) : method;
		this.candidates = candidates == null ? [] : candidates;
	}
}

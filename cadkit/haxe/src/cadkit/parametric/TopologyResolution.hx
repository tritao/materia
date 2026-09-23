package cadkit.parametric;

/** Result of conservative geometric topology matching. */
class TopologyResolution {
	public final index:Int;
	public final state:ReferenceState;

	public function new(index:Int, state:ReferenceState) {
		this.index = index;
		this.state = state;
	}
}

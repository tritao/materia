package cadkit.parametric;

import cadkit.Shape;

/** Result of mapping one topology snapshot through an operation history. */
class TopologyRemapResult {
	public final state:ReferenceState;
	public final shape:Null<Shape>;

	public function new(state:ReferenceState, shape:Null<Shape>) {
		this.state = state;
		this.shape = shape;
	}
}

package cadkit;

import CadKit;

/** Semantic wrapper around an owning vertex subshape handle. */
class Vertex {
	private final shape:Shape;

	public function new(shape:Shape) {
		this.shape = shape;
	}

	public function position():CadKit.Vec3 {
		return shape.position();
	}

	public function cloneShape():Shape {
		return shape.cloneShape();
	}

	public function sameAs(other:Vertex):Bool {
		return shape.sameAs(other.shape);
	}

	public function close():Bool {
		return shape.close();
	}
}

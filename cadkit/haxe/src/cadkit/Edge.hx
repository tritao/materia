package cadkit;

import CadKit;

/** Semantic wrapper around an owning edge subshape handle. */
class Edge {
	private final shape:Shape;

	public function new(shape:Shape) {
		this.shape = shape;
	}

	public function curveKind():CadKit.CurveKind {
		return shape.curveKind();
	}

	public function length():Float {
		return shape.edgeLength();
	}

	public function tangentAt(parameter:Float = 0.5):CadKit.Vec3 {
		return shape.tangentAt(parameter);
	}

	public function startPosition():CadKit.Vec3 {
		var vertex = shape.subshape(CadKit.ShapeKind.Vertex, 0);
		var position = vertex.position();
		vertex.close();
		return position;
	}

	public function cloneShape():Shape {
		return shape.cloneShape();
	}

	public function sameAs(other:Edge):Bool {
		return shape.sameAs(other.shape);
	}

	public function close():Bool {
		return shape.close();
	}
}

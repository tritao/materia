package cadkit;

import CadKit;

/** Semantic wrapper around an owning face subshape handle. */
class Face {
	private final shape:Shape;
	public final index:Int;

	public function new(shape:Shape, index:Int = -1) {
		this.shape = shape;
		this.index = index;
	}

	public function surfaceKind():CadKit.SurfaceKind {
		return shape.surfaceKind();
	}

	public function area():Float {
		return shape.faceArea();
	}

	public function center():CadKit.Vec3 {
		return shape.center();
	}

	public function normal():CadKit.Vec3 {
		return shape.faceNormal();
	}

	public function cloneShape():Shape {
		return shape.cloneShape();
	}

	public function sameAs(other:Face):Bool {
		return shape.sameAs(other.shape);
	}

	public function close():Bool {
		return shape.close();
	}
}

package cadkit.modeling;

import CadKit;
import cadkit.Geometry;

/** Immutable coordinates. Lengths use one consistent user-chosen unit; angles are radians. */
class Vector {
	public final x:Float;
	public final y:Float;
	public final z:Float;

	public function new(x:Float = 0, y:Float = 0, z:Float = 0) {
		if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
			throw "vector must be finite";
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public function add(v:Vector):Vector {
		return new Vector(x + v.x, y + v.y, z + v.z);
	}

	public function subtract(v:Vector):Vector {
		return new Vector(x - v.x, y - v.y, z - v.z);
	}

	public function scale(s:Float):Vector {
		return new Vector(x * s, y * s, z * s);
	}

	public function dot(v:Vector):Float {
		return x * v.x + y * v.y + z * v.z;
	}

	public function cross(v:Vector):Vector {
		return new Vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	public function length():Float {
		return Math.pow(dot(this), 0.5);
	}

	public function normalized():Vector {
		var n = length();
		if (!Math.isFinite(n) || n < 1e-12)
			throw "direction must be nonzero";
		return scale(1 / n);
	}

	public function native():CadKit.Vec3 {
		return Geometry.vec3(x, y, z);
	}

	public static function fromNative(v:CadKit.Vec3):Vector {
		return new Vector(v.get_x(), v.get_y(), v.get_z());
	}

	public static function X():Vector {
		return new Vector(1, 0, 0);
	}

	public static function Y():Vector {
		return new Vector(0, 1, 0);
	}

	public static function Z():Vector {
		return new Vector(0, 0, 1);
	}
}

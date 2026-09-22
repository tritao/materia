package cadkit.modeling;

class Axis {
	public final origin:Vector;
	public final direction:Vector;

	public function new(origin:Vector, direction:Vector) {
		this.origin = origin;
		this.direction = direction.normalized();
	}

	public static function X():Axis {
		return new Axis(new Vector(), Vector.X());
	}

	public static function Y():Axis {
		return new Axis(new Vector(), Vector.Y());
	}

	public static function Z():Axis {
		return new Axis(new Vector(), Vector.Z());
	}
}

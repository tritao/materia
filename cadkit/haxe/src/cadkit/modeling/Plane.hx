package cadkit.modeling;

/** Right-handed workplane. xDirection and normal must be perpendicular. */
class Plane {
	public final origin:Vector;
	public final xDirection:Vector;
	public final yDirection:Vector;
	public final normal:Vector;

	public function new(origin:Vector, xDirection:Vector, normal:Vector) {
		this.origin = origin;
		this.xDirection = xDirection.normalized();
		this.normal = normal.normalized();
		if (Math.abs(this.xDirection.dot(this.normal)) > 1e-10)
			throw "plane axes must be perpendicular";
		yDirection = this.normal.cross(this.xDirection).normalized();
	}

	public static function XY():Plane {
		return new Plane(new Vector(), Vector.X(), Vector.Z());
	}

	public static function XZ():Plane {
		return new Plane(new Vector(), Vector.X(), Vector.Y().scale(-1));
	}

	public static function YZ():Plane {
		return new Plane(new Vector(), Vector.Y(), Vector.X());
	}

	public function offset(distance:Float):Plane {
		return new Plane(origin.add(normal.scale(distance)), xDirection, normal);
	}

	public function toWorld(local:Vector):Vector {
		return origin.add(xDirection.scale(local.x)).add(yDirection.scale(local.y)).add(normal.scale(local.z));
	}

	public function toLocal(world:Vector):Vector {
		var v = world.subtract(origin);
		return new Vector(v.dot(xDirection), v.dot(yDirection), v.dot(normal));
	}

	public function location():Location {
		return new Location(this);
	}
}

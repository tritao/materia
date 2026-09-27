package cadkit.modeling;

/** Coordinate helpers for planar polygons. */
class Polygon {
	/** Regular polygon with a flat-to-flat width and one flat facing +X. */
	public static function regular(sides:Int, acrossFlats:Float):Array<Vector> {
		if (sides < 3 || !(acrossFlats > 0)) throw "Polygon needs at least three sides and a positive width";
		var radius = acrossFlats / (2 * Math.cos(Math.PI / sides));
		return [for (i in 0...sides) {
			var angle = Math.PI / sides + 2 * Math.PI * i / sides;
			new Vector(radius * Math.cos(angle), radius * Math.sin(angle));
		}];
	}
}

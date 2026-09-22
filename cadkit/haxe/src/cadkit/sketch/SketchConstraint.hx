package cadkit.sketch;

/** Generic stable constraint record. Factory methods document the supported reference combinations. */
class SketchConstraint {
	public final id:String;
	public final kind:String;
	public final first:String;
	public final second:Null<String>;
	public final third:Null<String>;
	public final value:Float;

	private function new(id:String, kind:String, first:String, second:Null<String>, third:Null<String>, value:Float) {
		this.id = id; this.kind = kind; this.first = first; this.second = second; this.third = third; this.value = value;
	}
	public static function raw(id:String, kind:String, first:String, second:Null<String>, third:Null<String>, value:Float):SketchConstraint
		return new SketchConstraint(id, kind, first, second, third, value);

	public static function fixed(id:String, point:String):SketchConstraint return new SketchConstraint(id, "fixed", point, null, null, 0);
	public static function coincident(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(id, "coincident", first, second, null, 0);
	public static function horizontal(id:String, line:String):SketchConstraint return new SketchConstraint(id, "horizontal", line, null, null, 0);
	public static function vertical(id:String, line:String):SketchConstraint return new SketchConstraint(id, "vertical", line, null, null, 0);
	public static function distance(id:String, first:String, second:String, value:Float):SketchConstraint return new SketchConstraint(id, "distance", first, second, null, value);
	public static function radius(id:String, entity:String, value:Float):SketchConstraint return new SketchConstraint(id, "radius", entity, null, null, value);
	public static function equal(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(id, "equal", first, second, null, 0);
	public static function parallel(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(id, "parallel", first, second, null, 0);
	public static function perpendicular(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(id, "perpendicular", first, second, null, 0);
	public static function angle(id:String, first:String, second:String, radians:Float):SketchConstraint return new SketchConstraint(id, "angle", first, second, null, radians);
	public static function concentric(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(id, "concentric", first, second, null, 0);
	public static function pointOn(id:String, point:String, entity:String):SketchConstraint return new SketchConstraint(id, "pointOn", point, entity, null, 0);
	/** Tangency supports line-circle, line-arc, circle-circle, circle-arc, and arc-arc. */
	public static function tangent(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(id, "tangent", first, second, null, 0);
	/** Reflect first point to second point across the referenced line. */
	public static function symmetric(id:String, first:String, second:String, axis:String):SketchConstraint return new SketchConstraint(id, "symmetric", first, second, axis, 0);
}

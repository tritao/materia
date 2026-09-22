package cadkit.sketch;

/** Stable authored entity. Lines reference point IDs; circles/arcs reference a center point. */
class SketchEntity {
	public final id:String;
	public final kind:String;
	public final first:String;
	public final second:Null<String>;
	public final radius:Float;
	public final startAngle:Float;
	public final endAngle:Float;
	public final clockwise:Bool;
	public final construction:Bool;

	private function new(id:String, kind:String, first:String, second:Null<String>, radius:Float,
		startAngle:Float, endAngle:Float, clockwise:Bool, construction:Bool) {
		this.id = id;
		this.kind = kind;
		this.first = first;
		this.second = second;
		this.radius = radius;
		this.startAngle = startAngle;
		this.endAngle = endAngle;
		this.clockwise = clockwise;
		this.construction = construction;
	}

	public static function line(id:String, start:String, end:String, construction:Bool = false):SketchEntity
		return new SketchEntity(id, "line", start, end, 0, 0, 0, false, construction);

	public static function circle(id:String, center:String, radius:Float, construction:Bool = false):SketchEntity
		return new SketchEntity(id, "circle", center, null, radius, 0, 0, false, construction);

	public static function arc(id:String, center:String, radius:Float, startAngle:Float, endAngle:Float,
		clockwise:Bool = false, construction:Bool = false):SketchEntity
		return new SketchEntity(id, "arc", center, null, radius, startAngle, endAngle, clockwise, construction);
}

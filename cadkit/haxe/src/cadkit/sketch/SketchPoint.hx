package cadkit.sketch;

/** Authored two-dimensional point. Coordinates are never changed by solving. */
class SketchPoint {
	public final id:String;
	public final x:Float;
	public final y:Float;

	public function new(id:String, x:Float, y:Float) {
		this.id = id;
		this.x = x;
		this.y = y;
	}
}

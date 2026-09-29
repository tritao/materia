package humankit.sim;

/** A floor polygon in session world metres. */
class HumanZone {
	public final id:String;
	public final polygon:Array<Array<Float>>;

	public function new(id:String, polygon:Array<Array<Float>>) {
		if (id.length == 0 || polygon.length < 3) throw "A zone needs an id and three vertices";
		this.id = id;
		this.polygon = [for (point in polygon) point.copy()];
	}

	public function contains(x:Float, y:Float):Bool {
		var inside = false;
		var previous = polygon.length - 1;
		for (index in 0...polygon.length) {
			var a = polygon[index], b = polygon[previous];
			if ((a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0])
				inside = !inside;
			previous = index;
		}
		return inside;
	}
}

package cadkit.sheetmetal;

/** Exact loops with authored segment order: outer CCW, holes CW. No implicit reversal renames edges. */
class WallOutline {
	final boundaries:Array<Array<Segment>>;

	public function new(outer:Array<Segment>, ?holes:Array<Array<Segment>>) {
		boundaries = [outer.copy()];
		if (holes != null)
			for (hole in holes)
				boundaries.push(hole.copy());
		for (i in 0...boundaries.length) {
			var loop = boundaries[i];
			if (loop.length == 0)
				throw new SheetDiagnosis("sheet.invalid-outline", "An outline loop cannot be empty");
			var area = 0.0;
			for (j in 0...loop.length) {
				if (loop[j].end.distance(loop[(j + 1) % loop.length].start) > 1e-8)
					throw new SheetDiagnosis("sheet.invalid-outline", "Outline segments must join in authored order");
				area += loop[j].area();
			}
			if ((i == 0 && area <= 1e-12) || (i > 0 && area >= -1e-12))
				throw new SheetDiagnosis("sheet.invalid-outline", "Outer loops must be CCW and holes CW, with nonzero area");
		}
		if (area() <= 1e-12)
			throw new SheetDiagnosis("sheet.invalid-outline", "Holes cannot consume the entire wall");
		OutlineGeometry.validate(boundaries);
	}
	public static function rectangle(width:Float, height:Float):WallOutline {
		if (!Math.isFinite(width) || !Math.isFinite(height) || width <= 0 || height <= 0)
			throw new SheetDiagnosis("sheet.invalid-outline", "Rectangle dimensions must be finite and positive");
		return polygon([new Point2(0, 0), new Point2(width, 0), new Point2(width, height), new Point2(0, height)]);
	}
	public static function polygon(points:Array<Point2>):WallOutline {
		var edges:Array<Segment> = [];
		for (i in 0...points.length)
			edges.push(Segment.line(points[i], points[(i + 1) % points.length]));
		return new WallOutline(edges);
	}
	public function loops():Array<Array<Segment>> {
		return [for (loop in boundaries) loop.copy()];
	}
	public function segment(loop:Int, index:Int):Segment {
		if (loop < 0 || loop >= boundaries.length || index < 0 || index >= boundaries[loop].length)
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "The referenced wall segment does not exist");
		return boundaries[loop][index];
	}
	public function area():Float {
		var result = 0.0;
		for (loop in boundaries)
			for (segment in loop)
				result += segment.area();
		return result;
	}
	public function withHole(hole:Array<Segment>):WallOutline {
		var holes = [for (i in 1...boundaries.length) boundaries[i]];
		holes.push(hole);
		return new WallOutline(boundaries[0], holes);
	}
}

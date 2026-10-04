package cadkit.sheetmetal;

/** Exact loops with authored segment order: outer CCW, holes CW. No implicit reversal renames edges. */
class WallOutline {
	final boundaries:Array<Array<Segment>>;
	final segmentIds:Array<Array<String>>;

	public function new(outer:Array<Segment>, ?holes:Array<Array<Segment>>, ?identities:Array<Array<String>>) {
		boundaries = [outer.copy()];
		if (holes != null)
			for (hole in holes)
				boundaries.push(hole.copy());
		if (identities != null && identities.length != boundaries.length)
			throw new SheetDiagnosis("sheet.invalid-outline-identity", "Segment identities must cover every loop");
		segmentIds = [];
		var seen:Map<String, Bool> = new Map();
		for (i in 0...boundaries.length) {
			var ids = identities == null ? [for (j in 0...boundaries[i].length) i + ":" + j] : identities[i].copy();
			if (ids.length != boundaries[i].length)
				throw new SheetDiagnosis("sheet.invalid-outline-identity", "Segment identities must cover every segment");
			for (id in ids) {
				if (id == null || id.length == 0 || seen.exists(id))
					throw new SheetDiagnosis("sheet.invalid-outline-identity", "Segment identities must be nonempty and unique", [id]);
				seen.set(id, true);
			}
			segmentIds.push(ids);
		}
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
	/** Authored identity for naming; it need not equal the segment's current contour index. */
	public function identity(loop:Int, index:Int):String {
		var checked = segment(loop, index);
		return segmentIds[loop][index];
	}
	public function identities():Array<Array<String>> {
		return [for (ids in segmentIds) ids.copy()];
	}
	/** Material membership away from the boundary, accounting for every authored hole. */
	public function contains(point:Point2):Bool {
		if (!OutlineGeometry.contains(boundaries[0], point)) return false;
		for (i in 1...boundaries.length) if (OutlineGeometry.contains(boundaries[i], point)) return false;
		return true;
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
		var ids = identities();
		ids.push([for (j in 0...hole.length) boundaries.length + ":" + j]);
		return new WallOutline(boundaries[0], holes, ids);
	}
}

package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.WallOutline;

/** Wall material decomposed into convex extrusions. Holes remain empty; arcs have an explicit chord error. */
class SheetPrismProfile {
	public final id:String;
	public final area:Float;
	public final curveError:Float;
	final solids:Array<ConvexPrism>;
	function new(id:String, area:Float, curveError:Float, solids:Array<ConvexPrism>) {
		this.id = id; this.area = area; this.curveError = curveError; this.solids = solids.copy();
	}
	public function pieces():Array<ConvexPrism> return solids.copy();

	/** Vertical material slabs preserve all loops without bridging a hole or taking a filled hull. */
	public static function of(id:String, outline:WallOutline, thickness:Float, maximumCurveError:Float = 0.01):SheetPrismProfile {
		if (id == null || id.length == 0 || !Math.isFinite(thickness) || thickness <= 0
			|| !Math.isFinite(maximumCurveError) || maximumCurveError <= 0)
			throw new SheetDiagnosis("press.profile-input", "Wall prisms need an identity, positive thickness and curve error");
		var edges:Array<Segment> = [], events:Array<Float> = [], error = 0.0;
		for (loop in outline.loops()) for (curve in loop) {
			if (curve.isLine()) edges.push(curve);
			else {
				var ratio = Math.min(1, maximumCurveError / curve.radius);
				var maximumAngle = 4 * Math.asin(Math.sqrt(ratio / 2));
				var estimate = Math.abs(curve.sweep) / maximumAngle;
				if (!Math.isFinite(estimate) || estimate > 1000000)
					throw new SheetDiagnosis("press.profile-resolution", "Requested arc error exceeds the one-million-chord profile budget", [id]);
				var count = Math.ceil(estimate);
				if (count < 4 && Math.abs(curve.sweep) > Math.PI) count = 4;
				if (count < 1) count = 1;
				var halfSagittaAngle = Math.abs(curve.sweep) / (4 * count);
				error = Math.max(error, 2 * curve.radius * Math.sin(halfSagittaAngle) * Math.sin(halfSagittaAngle));
				for (i in 0...count) edges.push(Segment.line(curve.point(i / count), curve.point((i + 1) / count)));
			}
		}
		for (edge in edges) { events.push(edge.start.x); events.push(edge.end.x); }
		events.sort(compare);
		var unique:Array<Float> = [];
		for (x in events) if (unique.length == 0 || x - unique[unique.length - 1] > 1e-10) unique.push(x);
		var pieces:Array<ConvexPrism> = [], area = 0.0;
		for (i in 0...(unique.length - 1)) {
			var left = unique[i], right = unique[i + 1], middle = (left + right) / 2;
			var active:Array<Segment> = [];
			for (edge in edges) if (middle > Math.min(edge.start.x, edge.end.x) && middle < Math.max(edge.start.x, edge.end.x)) active.push(edge);
			active.sort(function(a:Segment, b:Segment):Int { return compare(yAt(a, middle), yAt(b, middle)); });
			if (active.length % 2 != 0)
				throw new SheetDiagnosis("press.profile-decomposition", "Wall boundary has an odd number of slab crossings", [id]);
			var cursor = 0;
			while (cursor < active.length) {
				var lower = active[cursor++], upper = active[cursor++];
				var polygon = clean([new Point2(left, yAt(lower, left)), new Point2(right, yAt(lower, right)),
					new Point2(right, yAt(upper, right)), new Point2(left, yAt(upper, left))]);
				if (polygon.length < 3)
					throw new SheetDiagnosis("press.profile-decomposition", "Material slab collapsed below the geometric resolution", [id]);
				var twiceArea = 0.0;
				for (j in 0...polygon.length) {
					var next = polygon[(j + 1) % polygon.length];
					twiceArea += polygon[j].x * next.y - next.x * polygon[j].y;
				}
				area += Math.abs(twiceArea) / 2;
				pieces.push(new ConvexPrism(id + ":piece:" + pieces.length,
					[for (point in polygon) new Vector(point.x, point.y, 0)], new Vector(0, 0, -thickness)));
			}
		}
		if (pieces.length == 0)
			throw new SheetDiagnosis("press.profile-decomposition", "Wall has no decomposable material", [id]);
		return new SheetPrismProfile(id, area, error, pieces);
	}
	static function compare(a:Float, b:Float):Int return a < b ? -1 : a > b ? 1 : 0;
	static function yAt(edge:Segment, x:Float):Float
		return edge.start.y + (x - edge.start.x) * (edge.end.y - edge.start.y) / (edge.end.x - edge.start.x);
	static function clean(points:Array<Point2>):Array<Point2> {
		var result:Array<Point2> = [];
		for (point in points) if (result.length == 0 || result[result.length - 1].distance(point) > 1e-10) result.push(point);
		if (result.length > 1 && result[0].distance(result[result.length - 1]) <= 1e-10) { var removed = result.pop(); }
		return result;
	}
}

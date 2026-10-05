package machinekit.pressbrake.planning;

import cadkit.sheetmetal.OutlineGeometry;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.WallOutline;

/** Exact line/arc material overlap in a section. Boundary contact alone has no material area. */
class PressSectionQuery {
	public final tolerance:Float;
	public function new(tolerance:Float = 1e-8) {
		if (!Math.isFinite(tolerance) || tolerance < 0) throw "Section tolerance must be finite nonnegative mm";
		this.tolerance = tolerance;
	}
	public function overlap(a:WallOutline, b:WallOutline):Bool {
		var events:Array<Float> = [];
		var edgesA = edges(a), edgesB = edges(b);
		for (edge in edgesA) for (point in edge.extrema()) events.push(point.x);
		for (edge in edgesB) for (point in edge.extrema()) events.push(point.x);
		for (first in edgesA) for (second in edgesB)
			for (point in OutlineGeometry.intersections(first, second)) events.push(point.x);
		events.sort(compare);
		for (i in 0...(events.length - 1)) {
			if (events[i + 1] - events[i] <= tolerance) continue;
			var x = events[i] + (events[i + 1] - events[i]) / 2;
			var first = intervals(a, x), second = intervals(b, x);
			for (j in 0...Std.int(first.length / 2)) for (k in 0...Std.int(second.length / 2))
				if (Math.min(first[2 * j + 1], second[2 * k + 1]) - Math.max(first[2 * j], second[2 * k]) > tolerance) return true;
		}
		return false;
	}

	/** Alternating lower/upper material crossings; called strictly between curve events. */
	public function intervals(outline:WallOutline, x:Float):Array<Float> {
		var crossings:Array<Float> = [];
		for (loop in outline.loops()) for (edge in loop) {
			if (edge.isLine()) {
				if (x <= Math.min(edge.start.x, edge.end.x) || x >= Math.max(edge.start.x, edge.end.x)) continue;
				var fraction = (x - edge.start.x) / (edge.end.x - edge.start.x);
				crossings.push(edge.start.y + fraction * (edge.end.y - edge.start.y));
			} else {
				var center = edge.arcCenter(), distance = Math.abs(x - center.x);
				if (distance >= edge.radius) continue;
				var height = Math.sqrt((edge.radius - distance) * (edge.radius + distance));
				for (y in [center.y - height, center.y + height]) {
					var fraction = edge.fractionAt(new Point2(x, y));
					if (fraction >= 0 && fraction <= 1 + 1e-10) crossings.push(y);
				}
			}
		}
		crossings.sort(compare);
		if (crossings.length % 2 != 0)
			throw new SheetDiagnosis("press.section-crossings", "Exact section has an odd material crossing count at " + x);
		return crossings;
	}
	/** Material projection onto X inside an open Y strip; holes are filled only if material exists elsewhere in that strip. */
	public function projection(outline:WallOutline, low:Float, high:Float):Array<Float> {
		if (!Math.isFinite(low) || !Math.isFinite(high) || high - low <= tolerance) return [];
		var source = edges(outline), events:Array<Float> = [];
		for (edge in source) for (point in edge.extrema()) events.push(point.x);
		events.sort(compare);
		var left = events[0] - 1, right = events[events.length - 1] + 1;
		for (y in [low, high]) {
			var crossing = Segment.line(new Point2(left, y), new Point2(right, y));
			for (edge in source) for (point in OutlineGeometry.intersections(edge, crossing)) events.push(point.x);
		}
		events.sort(compare);
		var result:Array<Float> = [];
		for (i in 0...(events.length - 1)) {
			if (events[i + 1] - events[i] <= tolerance) continue;
			var crossings = intervals(outline, events[i] + (events[i + 1] - events[i]) / 2), material = false;
			for (j in 0...Std.int(crossings.length / 2))
				if (Math.min(crossings[2 * j + 1], high) - Math.max(crossings[2 * j], low) > tolerance) material = true;
			if (!material) continue;
			if (result.length > 0 && Math.abs(result[result.length - 1] - events[i]) <= tolerance)
				result[result.length - 1] = events[i + 1];
			else { result.push(events[i]); result.push(events[i + 1]); }
		}
		return result;
	}
	static function edges(outline:WallOutline):Array<Segment> {
		var result:Array<Segment> = [];
		for (loop in outline.loops()) for (edge in loop) result.push(edge);
		return result;
	}
	static function compare(a:Float, b:Float):Int return a < b ? -1 : a > b ? 1 : 0;
}

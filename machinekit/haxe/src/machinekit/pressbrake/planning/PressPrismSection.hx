package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.WallOutline;

/** Exact Y/Z projection of a convex prism clipped to a finite open bed interval. */
class PressPrismSection {
	public static function of(prism:ConvexPrism, first:Float, last:Float, tolerance:Float = 1e-8):Null<WallOutline> {
		if (!Math.isFinite(first) || !Math.isFinite(last) || last <= first) throw "Prism section needs a finite positive bed interval";
		var vertices = prism.vertices(), count = Std.int(vertices.length / 2), points:Array<Point2> = [];
		var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
		for (point in vertices) { low = Math.min(low, point.x); high = Math.max(high, point.x); }
		if (Math.min(high, last) - Math.max(low, first) <= tolerance) return null;
		for (point in vertices) if (point.x >= first && point.x <= last) points.push(new Point2(point.y, point.z));
		for (i in 0...count) {
			crossings(points, vertices[i], vertices[(i + 1) % count], first, last);
			crossings(points, vertices[i + count], vertices[(i + 1) % count + count], first, last);
			crossings(points, vertices[i], vertices[i + count], first, last);
		}
		points.sort(function(a:Point2, b:Point2):Int return a.x < b.x ? -1 : a.x > b.x ? 1 : a.y < b.y ? -1 : a.y > b.y ? 1 : 0);
		var unique:Array<Point2> = [];
		for (point in points) if (unique.length == 0 || point.distance(unique[unique.length - 1]) > 1e-10) unique.push(point);
		if (unique.length < 3) return null;
		var lower:Array<Point2> = [], upper:Array<Point2> = [];
		for (point in unique) append(lower, point);
		for (i in 0...unique.length) append(upper, unique[unique.length - 1 - i]);
		var removedLower = lower.pop(), removedUpper = upper.pop();
		for (point in upper) lower.push(point);
		if (lower.length < 3) return null;
		return WallOutline.polygon(lower);
	}
	static function append(hull:Array<Point2>, point:Point2):Void {
		while (hull.length >= 2) {
			var a = hull[hull.length - 2], b = hull[hull.length - 1];
			if ((b.x - a.x) * (point.y - b.y) - (b.y - a.y) * (point.x - b.x) > 1e-12) break;
			var removed = hull.pop();
		}
		hull.push(point);
	}
	static function crossings(points:Array<Point2>, a:Vector, b:Vector, first:Float, last:Float):Void {
		if (a.x == b.x) return;
		for (x in [first, last]) {
			var fraction = (x - a.x) / (b.x - a.x);
			if (fraction >= 0 && fraction <= 1)
				points.push(new Point2(a.y + fraction * (b.y - a.y), a.z + fraction * (b.z - a.z)));
		}
	}
}

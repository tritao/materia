package machinekit.pressbrake;

/** A simple tooling section. Convex pieces preserve its relief instead of filling it with a hull. */
class ToolProfilePolygon {
	final outline:Array<ToolProfilePoint>;
	public final area:Float;
	static inline var tolerance:Float = 1e-9;

	public function new(points:Array<ToolProfilePoint>) {
		if (points == null || points.length < 3) throw "Tool profile needs at least three points";
		outline = points.copy();
		var twiceArea = 0.0;
		for (i in 0...outline.length) {
			var a = outline[i], b = outline[(i + 1) % outline.length];
			if (a == null || b == null) throw "Tool profile has a null point";
			if (Math.abs(a.y - b.y) <= tolerance && Math.abs(a.z - b.z) <= tolerance)
				throw "Tool profile has a zero-length edge";
			twiceArea += a.y * b.z - b.y * a.z;
		}
		if (Math.abs(twiceArea) <= tolerance) throw "Tool profile has zero area";
		for (i in 0...outline.length) for (j in i + 1...outline.length) {
			if (j == i + 1 || (i == 0 && j == outline.length - 1)) continue;
			if (intersects(outline[i], outline[(i + 1) % outline.length], outline[j], outline[(j + 1) % outline.length]))
				throw "Tool profile has intersecting edges";
		}
		if (twiceArea < 0) outline.reverse();
		area = Math.abs(twiceArea) / 2;
	}

	public function points():Array<ToolProfilePoint> return outline.copy();

	/** Ear clipping gives exact convex triangles of the authored polygon. */
	public function convexPieces():Array<Array<ToolProfilePoint>> {
		var remaining = outline.copy();
		var result:Array<Array<ToolProfilePoint>> = [];
		while (remaining.length > 3) {
			var found = false;
			for (i in 0...remaining.length) {
				var a = remaining[(i + remaining.length - 1) % remaining.length];
				var b = remaining[i], c = remaining[(i + 1) % remaining.length];
				if (cross(a, b, c) <= tolerance) continue;
				var occupied = false;
				for (p in remaining) {
					if (p == a || p == b || p == c) continue;
					if (cross(a, b, p) >= -tolerance && cross(b, c, p) >= -tolerance && cross(c, a, p) >= -tolerance)
						occupied = true;
				}
				if (occupied) continue;
				result.push([a, b, c]);
				remaining.splice(i, 1);
				found = true;
				break;
			}
			if (!found) throw "Tool profile cannot be decomposed into convex pieces";
		}
		if (cross(remaining[0], remaining[1], remaining[2]) <= tolerance)
			throw "Tool profile decomposition has a degenerate final piece";
		result.push(remaining);
		return result;
	}

	static function cross(a:ToolProfilePoint, b:ToolProfilePoint, c:ToolProfilePoint):Float
		return (b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y);

	static function onSegment(a:ToolProfilePoint, b:ToolProfilePoint, p:ToolProfilePoint):Bool
		return Math.abs(cross(a, b, p)) <= tolerance
			&& p.y >= Math.min(a.y, b.y) - tolerance && p.y <= Math.max(a.y, b.y) + tolerance
			&& p.z >= Math.min(a.z, b.z) - tolerance && p.z <= Math.max(a.z, b.z) + tolerance;

	static function intersects(a:ToolProfilePoint, b:ToolProfilePoint, c:ToolProfilePoint, d:ToolProfilePoint):Bool {
		var abC = cross(a, b, c), abD = cross(a, b, d), cdA = cross(c, d, a), cdB = cross(c, d, b);
		return ((abC > tolerance && abD < -tolerance || abC < -tolerance && abD > tolerance)
			&& (cdA > tolerance && cdB < -tolerance || cdA < -tolerance && cdB > tolerance))
			|| onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b);
	}
}

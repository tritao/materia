package materia.casting;

import materia.casting.CastingPrism.CastingPoint2;

/** Exact line intersections; tolerance is used only for coincident geometry. */
class CastingPolygons {
	public static function halfPlane(points:Array<CastingPoint2>, a:Float, b:Float, c:Float):Array<CastingPoint2> {
		var out:Array<CastingPoint2> = [];
		if (points.length == 0)
			return out;
		for (i in 0...points.length) {
			var p = points[i], q = points[(i + 1) % points.length], u = a * p.x + b * p.y - c, v = a * q.x + b * q.y - c;
			if (u <= 1e-9)
				out.push({x: p.x, y: p.y});
			if ((u < -1e-9 && v > 1e-9) || (u > 1e-9 && v < -1e-9)) {
				var t = u / (u - v);
				out.push({x: p.x + t * (q.x - p.x), y: p.y + t * (q.y - p.y)});
			}
		}
		return clean(out);
	}

	public static function clean(points:Array<CastingPoint2>):Array<CastingPoint2> {
		var out:Array<CastingPoint2> = [];
		for (p in points)
			if (out.length == 0 || Math.abs(p.x - out[out.length - 1].x) + Math.abs(p.y - out[out.length - 1].y) > 1e-9)
				out.push(p);
		if (out.length > 1 && Math.abs(out[0].x - out[out.length - 1].x) + Math.abs(out[0].y - out[out.length - 1].y) < 1e-9)
			out.pop();
		var changed = true;
		while (changed && out.length >= 3) {
			changed = false;
			for (i in 0...out.length) {
				var a = out[(i + out.length - 1) % out.length], b = out[i], c = out[(i + 1) % out.length];
				if (Math.abs((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) < 1e-9) {
					out.splice(i, 1);
					changed = true;
					break;
				}
			}
		}
		return out;
	}

	public static function area(p:Array<CastingPoint2>):Float {
		var twice = 0.0;
		for (i in 0...p.length)
			twice += p[i].x * p[(i + 1) % p.length].y - p[(i + 1) % p.length].x * p[i].y;
		return twice / 2;
	}
}

package materia.tooling.panel;

import materia.casting.CastingPrism.CastingPoint2;
import materia.tooling.panel.PanelPlan.PanelSegment;
import materia.tooling.panel.PanelPlan.PanelCurveKind;

/** Exact line/circle predicates. No tessellation and no cutter compensation. */
class PanelGeometry {
	public static inline var TOL = 1e-7;

	static function cross(ax:Float, ay:Float, bx:Float, by:Float):Float
		return ax * by - ay * bx;

	static function angle(a:Float):Float {
		var v = a % (2 * Math.PI);
		return v < 0 ? v + 2 * Math.PI : v;
	}

	public static function on(s:PanelSegment, p:CastingPoint2):Bool {
		var c = s.center;
		if (c == null) {
			var dx = s.end.x - s.start.x, dy = s.end.y - s.start.y;
			return Math.abs(cross(dx, dy, p.x - s.start.x, p.y - s.start.y)) <= TOL * Math.max(1, PanelContours.distance(s.start, s.end))
				&& p.x >= Math.min(s.start.x, s.end.x) - TOL
				&& p.x <= Math.max(s.start.x, s.end.x) + TOL
				&& p.y >= Math.min(s.start.y, s.end.y) - TOL
				&& p.y <= Math.max(s.start.y, s.end.y) + TOL;
		}
		if (Math.abs(PanelContours.distance(c, p) - PanelContours.distance(c, s.start)) > TOL)
			return false;
		var a = Math.atan2(s.start.y - c.y, s.start.x - c.x),
			b = Math.atan2(p.y - c.y, p.x - c.x);
		return angle(s.sweep > 0 ? b - a : a - b) <= Math.abs(s.sweep) + 1e-9 || PanelContours.distance(p, s.start) < TOL;
	}

	public static function intersections(a:PanelSegment, b:PanelSegment):Array<CastingPoint2> {
		var candidates:Array<CastingPoint2> = [], ca = a.center, cb = b.center;
		if (ca == null && cb == null) {
			var ax = a.end.x - a.start.x,
				ay = a.end.y - a.start.y,
				bx = b.end.x - b.start.x,
				by = b.end.y - b.start.y;
			var d = cross(ax, ay, bx, by);
			if (Math.abs(d) > 1e-12) {
				var t = cross(b.start.x - a.start.x, b.start.y - a.start.y, bx, by) / d;
				candidates.push({x: a.start.x + t * ax, y: a.start.y + t * ay});
			} else
				for (p in [a.start, a.end, b.start, b.end])
					candidates.push(p);
		} else if (ca == null || cb == null) {
			var l = ca == null ? a : b,
				circle = ca == null ? b : a,
				c = circle.center;
			if (c == null)
				throw "Missing circle center";
			var dx = l.end.x - l.start.x,
				dy = l.end.y - l.start.y,
				fx = l.start.x - c.x,
				fy = l.start.y - c.y;
			var aa = dx * dx + dy * dy,
				bb = 2 * (fx * dx + fy * dy),
				r = PanelContours.distance(c, circle.start),
				cc = fx * fx + fy * fy - r * r;
			var disc = bb * bb - 4 * aa * cc;
			if (disc >= -TOL && aa > 0)
				for (t in [
					(-bb - Math.sqrt(Math.max(0, disc))) / (2 * aa),
					(-bb + Math.sqrt(Math.max(0, disc))) / (2 * aa)
				])
					candidates.push({x: l.start.x + t * dx, y: l.start.y + t * dy});
		} else {
			var d = PanelContours.distance(ca, cb),
				r = PanelContours.distance(ca, a.start),
				q = PanelContours.distance(cb, b.start);
			if (d < TOL && Math.abs(r - q) < TOL) {
				candidates = [
					a.start,
					a.end,
					b.start,
					b.end,
					PanelContours.point(a, 0.5),
					PanelContours.point(b, 0.5)
				];
			} else if (d > TOL && d <= r + q + TOL && d >= Math.abs(r - q) - TOL) {
				var x = (r * r - q * q + d * d) / (2 * d),
					h = Math.sqrt(Math.max(0, r * r - x * x)),
					ux = (cb.x - ca.x) / d,
					uy = (cb.y - ca.y) / d;
				for (sign in [-1.0, 1.0])
					candidates.push({x: ca.x + x * ux - sign * h * uy, y: ca.y + x * uy + sign * h * ux});
			}
		}
		var result:Array<CastingPoint2> = [];
		for (p in candidates)
			if (on(a, p) && on(b, p) && result.filter(q -> PanelContours.distance(p, q) < TOL).length == 0)
				result.push(p);
		return result;
	}

	/** Exact line/arc minimum boundary distance, including interior stationary points. */
	public static function distanceBetween(a:Array<PanelSegment>, b:Array<PanelSegment>):Float {
		if (inside(a, b[0].start.x, b[0].start.y) || inside(b, a[0].start.x, a[0].start.y))
			return 0;
		var minimum = 1e100;
		function pointDistance(p:CastingPoint2, s:PanelSegment):Float {
			var c = s.center;
			var best = Math.min(PanelContours.distance(p, s.start), PanelContours.distance(p, s.end));
			if (c == null) {
				var dx = s.end.x - s.start.x, dy = s.end.y - s.start.y;
				var t = ((p.x - s.start.x) * dx + (p.y - s.start.y) * dy) / (dx * dx + dy * dy);
				t = Math.max(0, Math.min(1, t));
				return Math.min(best, PanelContours.distance(p, {x: s.start.x + t * dx, y: s.start.y + t * dy}));
			}
			var length = PanelContours.distance(p, c);
			if (length > TOL) {
				var r = PanelContours.distance(c, s.start),
					q = {x: c.x + (p.x - c.x) * r / length, y: c.y + (p.y - c.y) * r / length};
				if (on(s, q))
					best = Math.min(best, PanelContours.distance(p, q));
			}
			return best;
		}
		for (x in a)
			for (y in b) {
				if (intersections(x, y).length > 0)
					return 0;
				for (p in [x.start, x.end])
					minimum = Math.min(minimum, pointDistance(p, y));
				for (p in [y.start, y.end])
					minimum = Math.min(minimum, pointDistance(p, x));
				var cx = x.center, cy = y.center;
				if (cx != null && cy != null) {
					var length = PanelContours.distance(cx, cy);
					if (length > TOL)
						for (sx in [-1.0, 1.0])
							for (sy in [-1.0, 1.0]) {
								var rx = PanelContours.distance(cx, x.start),
									ry = PanelContours.distance(cy, y.start);
								var px = {x: cx.x + sx * rx * (cy.x - cx.x) / length, y: cx.y + sx * rx * (cy.y - cx.y) / length};
								var py = {x: cy.x + sy * ry * (cy.x - cx.x) / length, y: cy.y + sy * ry * (cy.y - cx.y) / length};
								if (on(x, px) && on(y, py))
									minimum = Math.min(minimum, PanelContours.distance(px, py));
							}
				} else if (cx != null || cy != null) {
					var line = cx == null ? x : y, arc = cx == null ? y : x, c = arc.center;
					if (c == null)
						throw "Missing arc center";
					var dx = line.end.x - line.start.x,
						dy = line.end.y - line.start.y,
						length = Math.sqrt(dx * dx + dy * dy),
						r = PanelContours.distance(c, arc.start);
					for (sign in [-1.0, 1.0]) {
						var p = {x: c.x - sign * r * dy / length, y: c.y + sign * r * dx / length};
						if (on(arc, p))
							minimum = Math.min(minimum, pointDistance(p, line));
					}
				}
			}
		return minimum;
	}

	public static function validate(p:Array<PanelSegment>):Void {
		if (p == null || p.length < 2 || p.length > 512)
			throw "Invalid exact panel contour count";
		for (i in 0...p.length) {
			var s = p[i], c = s.center;
			for (v in [s.start.x, s.start.y, s.end.x, s.end.y, s.sweep])
				if (!Math.isFinite(v))
					throw "Nonfinite panel contour";
			if (s.kind == Line) {
				if (c != null || s.sweep != 0 || PanelContours.distance(s.start, s.end) < TOL)
					throw "Invalid panel line";
			} else if (s.kind == Arc) {
				if (c == null
					|| !Math.isFinite(c.x)
					|| !Math.isFinite(c.y)
					|| Math.abs(s.sweep) < 1e-10
					|| Math.abs(s.sweep) >= 2 * Math.PI - 1e-12
					|| PanelContours.distance(c, s.start) < TOL
					|| PanelContours.distance(PanelContours.point(s, 1), s.end) > TOL)
					throw "Invalid panel arc";
			} else
				throw "Unknown panel curve kind";
			if (PanelContours.distance(s.end, p[(i + 1) % p.length].start) > TOL)
				throw "Open panel contour";
			for (j in i + 1...p.length)
				for (hit in intersections(s, p[j])) {
					var adjacent = j == i + 1 || (i == 0 && j == p.length - 1);
					var endpoints = (PanelContours.distance(hit, s.start) < TOL || PanelContours.distance(hit, s.end) < TOL)
						&& (PanelContours.distance(hit, p[j].start) < TOL || PanelContours.distance(hit, p[j].end) < TOL);
					if (!adjacent || !endpoints)
						throw "Panel contour self-intersects";
				}
		}
		var signedArea = area(p);
		if (!Math.isFinite(signedArea) || signedArea <= TOL)
			throw "Panel contour must have finite positive CCW area";
	}

	public static function area(p:Array<PanelSegment>):Float {
		var total = 0.0;
		for (s in p) {
			var c = s.center;
			total += c == null ? (s.start.x * s.end.y - s.end.x * s.start.y) / 2 : (c.x * (s.end.y - s.start.y) - c.y * (s.end.x - s.start.x)
				+ Math.pow(PanelContours.distance(c, s.start), 2) * s.sweep) / 2;
		}
		return total;
	}

	public static function extrema(s:PanelSegment):Array<CastingPoint2> {
		var result = [s.start, s.end], c = s.center;
		if (c != null)
			for (i in 0...4) {
				var r = PanelContours.distance(c, s.start),
					p = {x: c.x + r * Math.cos(i * Math.PI / 2), y: c.y + r * Math.sin(i * Math.PI / 2)};
				if (on(s, p))
					result.push(p);
			}
		return result;
	}

	public static function bounds(p:Array<PanelSegment>):{
		x:Float,
		y:Float,
		width:Float,
		height:Float
	} {
		var x = Math.POSITIVE_INFINITY, y = x, xx = Math.NEGATIVE_INFINITY, yy = xx;
		for (s in p)
			for (v in extrema(s)) {
				x = Math.min(x, v.x);
				y = Math.min(y, v.y);
				xx = Math.max(xx, v.x);
				yy = Math.max(yy, v.y);
			}
		return {
			x: x,
			y: y,
			width: xx - x,
			height: yy - y
		};
	}

	public static function inside(p:Array<PanelSegment>, x:Float, y:Float):Bool {
		var query = {x: x, y: y}, hits = 0;
		for (s in p) {
			if (on(s, query))
				return true;
			var c = s.center;
			if (c == null) {
				if ((s.start.y > y) != (s.end.y > y) && x < (s.end.x - s.start.x) * (y - s.start.y) / (s.end.y - s.start.y) + s.start.x)
					hits++;
			} else {
				// Split at exact Y extrema, then apply the same half-open crossing rule.
				var fractions = [0.0, 1.0],
					a = Math.atan2(s.start.y - c.y, s.start.x - c.x),
					r = PanelContours.distance(c, s.start);
				for (t in [Math.PI / 2, 3 * Math.PI / 2]) {
					var delta = angle(s.sweep > 0 ? t - a : a - t),
						f = delta / Math.abs(s.sweep);
					if (f > 1e-10 && f < 1 - 1e-10)
						fractions.push(f);
				}
				fractions.sort(Reflect.compare);
				for (i in 0...fractions.length - 1) {
					var u = PanelContours.point(s, fractions[i]),
						v = PanelContours.point(s, fractions[i + 1]);
					if ((u.y > y) != (v.y > y)) {
						var mid = PanelContours.point(s, (fractions[i] + fractions[i + 1]) / 2),
							dx = Math.sqrt(Math.max(0, r * r - (y - c.y) * (y - c.y)));
						if (x < c.x + (mid.x >= c.x ? dx : -dx))
							hits++;
					}
				}
			}
		}
		return hits % 2 == 1;
	}

	public static function containsRect(p:Array<PanelSegment>, x:Float, y:Float, w:Float, h:Float):Bool {
		var box = PanelContours.rectangle(x, y, w, h);
		for (s in box)
			if (!inside(p, s.start.x, s.start.y))
				return false;
		for (s in p) {
			for (v in extrema(s))
				if (v.x > x + TOL && v.x < x + w - TOL && v.y > y + TOL && v.y < y + h - TOL)
					return false;
			for (edge in box)
				for (v in intersections(s, edge)) {
					var c = s.center;
					var tx = c == null ? s.end.x - s.start.x : -(v.y - c.y);
					var ty = c == null ? s.end.y - s.start.y : v.x - c.x;
					if (edge.start.x == edge.end.x ? Math.abs(tx) > TOL : Math.abs(ty) > TOL)
						return false;
				}
		}
		return true;
	}
}

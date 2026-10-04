package cadkit.sheetmetal;

/** Analytic line/arc intersections and winding used to validate exact sheet regions. */
class OutlineGeometry {
	static inline var EPS:Float = 1e-8;

	public static function validate(loops:Array<Array<Segment>>):Void {
		for (loop in loops)
			for (i in 0...loop.length)
				for (j in (i + 1)...loop.length) {
					var adjacent = j == i + 1 || (i == 0 && j == loop.length - 1);
					for (p in intersections(loop[i], loop[j])) {
						var sharedEnd = (p.distance(loop[i].end) < EPS && p.distance(loop[j].start) < EPS)
							|| (p.distance(loop[j].end) < EPS && p.distance(loop[i].start) < EPS);
						if (!adjacent || !sharedEnd)
							throw new SheetDiagnosis("sheet.invalid-outline", "Outline segments cross or overlap");
					}
				}
		for (i in 1...loops.length) {
			if (!contains(loops[0], loops[i][0].start))
				throw new SheetDiagnosis("sheet.invalid-outline", "A hole lies outside its wall");
			for (j in 0...i) {
				for (a in loops[i])
					for (b in loops[j])
						if (intersections(a, b).length > 0)
							throw new SheetDiagnosis("sheet.invalid-outline", "A hole touches or crosses another boundary");
				if (j > 0 && (contains(loops[j], loops[i][0].start) || contains(loops[i], loops[j][0].start)))
					throw new SheetDiagnosis("sheet.invalid-outline", "Wall holes must not overlap or nest");
			}
		}
	}

	public static function contains(loop:Array<Segment>, point:Point2):Bool {
		var winding = 0;
		for (s in loop) {
			if (s.isLine()) {
				if ((s.start.y <= point.y && s.end.y > point.y) || (s.end.y <= point.y && s.start.y > point.y)) {
					var x = s.start.x + (point.y - s.start.y) * (s.end.x - s.start.x) / (s.end.y - s.start.y);
					if (x > point.x) winding += s.end.y > s.start.y ? 1 : -1;
				}
			} else {
				var center = s.arcCenter();
				var sine = (point.y - center.y) / s.radius;
				if (Math.abs(sine) >= 1) continue;
				var a = Math.asin(sine);
				for (angle in [a, Math.PI - a]) {
					var f = arcFraction(s, angle);
					if (f < -1e-12 || f > 1 + 1e-12) continue;
					var direction = Math.cos(angle) * s.sweep;
					var fullCircle = Math.abs(s.sweep) >= 2 * Math.PI - 1e-12;
					if (!fullCircle && ((f <= 1e-12 && direction < 0) || (f >= 1 - 1e-12 && direction > 0))) continue;
					if (center.x + s.radius * Math.cos(angle) > point.x)
						winding += direction > 0 ? 1 : -1;
				}
			}
		}
		return winding != 0;
	}

	static function arcFraction(s:Segment, angle:Float):Float {
		var delta = (s.sweep > 0 ? angle - s.startAngle : s.startAngle - angle) % (2 * Math.PI);
		if (delta < 0) delta += 2 * Math.PI;
		if (delta > 2 * Math.PI - 1e-12) delta = 0;
		return delta / Math.abs(s.sweep);
	}
	static function onArc(s:Segment, p:Point2):Bool {
		var center = s.arcCenter();
		return arcFraction(s, Math.atan2(p.y - center.y, p.x - center.x)) <= 1 + 1e-10;
	}
	static function cross(x:Float, y:Float, u:Float, v:Float):Float { return x * v - y * u; }
	static function onLine(s:Segment, p:Point2):Bool {
		var dx = s.end.x - s.start.x;
		var dy = s.end.y - s.start.y;
		var t = ((p.x - s.start.x) * dx + (p.y - s.start.y) * dy) / (dx * dx + dy * dy);
		return Math.abs(cross(p.x - s.start.x, p.y - s.start.y, dx, dy)) <= EPS * s.length()
			&& t >= -1e-10 && t <= 1 + 1e-10;
	}
	public static function intersections(a:Segment, b:Segment):Array<Point2> {
		if (a.isLine() && b.isLine()) {
			var dx = a.end.x - a.start.x;
			var dy = a.end.y - a.start.y;
			var ex = b.end.x - b.start.x;
			var ey = b.end.y - b.start.y;
			var determinant = cross(dx, dy, ex, ey);
			if (Math.abs(determinant) < 1e-12 * a.length() * b.length()) {
				var result:Array<Point2> = [];
				for (p in [a.start, a.end]) if (onLine(b, p)) result.push(p);
				for (p in [b.start, b.end]) if (onLine(a, p)) result.push(p);
				// An interior sample identifies collinear overlap even when its endpoints are adjacent.
				for (p in [a.point(0.5), b.point(0.5)]) if (onLine(a, p) && onLine(b, p)) result.push(p);
				return result;
			}
			var px = b.start.x - a.start.x;
			var py = b.start.y - a.start.y;
			var t = cross(px, py, ex, ey) / determinant;
			var u = cross(px, py, dx, dy) / determinant;
			return t >= -1e-10 && t <= 1 + 1e-10 && u >= -1e-10 && u <= 1 + 1e-10
				? [new Point2(a.start.x + t * dx, a.start.y + t * dy)] : [];
		}
		if (a.isLine() || b.isLine()) return lineArc(a.isLine() ? a : b, a.isLine() ? b : a);
		var ca = a.arcCenter();
		var cb = b.arcCenter();
		var distance = ca.distance(cb);
		if (distance < EPS && Math.abs(a.radius - b.radius) < EPS) {
			var result:Array<Point2> = [];
			for (p in [a.start, a.end, a.point(0.25), a.point(0.5), a.point(0.75)]) if (onArc(b, p)) result.push(p);
			for (p in [b.start, b.end, b.point(0.25), b.point(0.5), b.point(0.75)]) if (onArc(a, p)) result.push(p);
			return result;
		}
		if (distance < EPS || distance > a.radius + b.radius + EPS || distance < Math.abs(a.radius - b.radius) - EPS) return [];
		var along = (a.radius * a.radius - b.radius * b.radius + distance * distance) / (2 * distance);
		var height = Math.sqrt(Math.max(0, a.radius * a.radius - along * along));
		var dx = (cb.x - ca.x) / distance;
		var dy = (cb.y - ca.y) / distance;
		var result:Array<Point2> = [];
		for (sign in [-1.0, 1.0]) {
			var p = new Point2(ca.x + along * dx - sign * height * dy, ca.y + along * dy + sign * height * dx);
			if (onArc(a, p) && onArc(b, p)) result.push(p);
		}
		return result;
	}
	static function lineArc(line:Segment, arc:Segment):Array<Point2> {
		var center = arc.arcCenter();
		var dx = line.end.x - line.start.x;
		var dy = line.end.y - line.start.y;
		var px = line.start.x - center.x;
		var py = line.start.y - center.y;
		var aa = dx * dx + dy * dy;
		var bb = 2 * (dx * px + dy * py);
		var cc = px * px + py * py - arc.radius * arc.radius;
		var discriminant = bb * bb - 4 * aa * cc;
		if (discriminant < -EPS) return [];
		var root = Math.sqrt(Math.max(0, discriminant));
		var result:Array<Point2> = [];
		for (t in [(-bb - root) / (2 * aa), (-bb + root) / (2 * aa)])
			if (t >= -1e-10 && t <= 1 + 1e-10) {
				var p = new Point2(line.start.x + t * dx, line.start.y + t * dy);
				if (onArc(arc, p)) result.push(p);
			}
		return result;
	}
}

package cadkit.sheetmetal;

/** An exact directed line or circular arc. Positive arc sweep is counterclockwise. */
class Segment {
	public final start:Point2;
	public final end:Point2;
	public final center:Null<Point2>;
	public final radius:Float;
	public final startAngle:Float;
	public final sweep:Float;

	function new(start:Point2, end:Point2, center:Null<Point2>, radius:Float, startAngle:Float, sweep:Float) {
		this.start = start;
		this.end = end;
		this.center = center;
		this.radius = radius;
		this.startAngle = startAngle;
		this.sweep = sweep;
	}
	public static function line(start:Point2, end:Point2):Segment {
		if (start.distance(end) <= 1e-9)
			throw new SheetDiagnosis("sheet.invalid-outline", "A line must have positive length");
		return new Segment(start, end, null, 0, 0, 0);
	}
	public static function arc(center:Point2, radius:Float, startAngle:Float, sweep:Float):Segment {
		if (!Math.isFinite(radius) || radius <= 0 || !Math.isFinite(startAngle) || !Math.isFinite(sweep)
			|| Math.abs(sweep) <= 1e-12 || Math.abs(sweep) > 2 * Math.PI)
			throw new SheetDiagnosis("sheet.invalid-outline", "An arc needs a positive radius and a finite sweep up to one turn");
		return new Segment(new Point2(center.x + radius * Math.cos(startAngle), center.y + radius * Math.sin(startAngle)),
			new Point2(center.x + radius * Math.cos(startAngle + sweep), center.y + radius * Math.sin(startAngle + sweep)),
			center, radius, startAngle, sweep);
	}
	public function isLine():Bool {
		return center == null;
	}
	public function arcCenter():Point2 {
		var value = center;
		if (value == null) throw new SheetDiagnosis("sheet.not-an-arc", "A straight segment has no circle centre");
		return value;
	}
	public function length():Float {
		return isLine() ? start.distance(end) : radius * Math.abs(sweep);
	}
	/** Signed Green integral, including the circular sector rather than its chord. */
	public function area():Float {
		if (center == null)
			return (start.x * end.y - end.x * start.y) / 2;
		var c = arcCenter();
		return (c.x * (end.y - start.y) - c.y * (end.x - start.x) + radius * radius * sweep) / 2;
	}
	public function point(fraction:Float):Point2 {
		if (!Math.isFinite(fraction) || fraction < 0 || fraction > 1)
			throw new SheetDiagnosis("sheet.invalid-outline", "Segment fraction must lie in [0, 1]");
		if (center == null)
			return new Point2(start.x + (end.x - start.x) * fraction, start.y + (end.y - start.y) * fraction);
		var c = arcCenter();
		var a = startAngle + sweep * fraction;
		return new Point2(c.x + radius * Math.cos(a), c.y + radius * Math.sin(a));
	}
	/** Parameter of a point already known to lie on this curve; full-circle endpoints use zero. */
	public function fractionAt(point:Point2):Float {
		if (isLine()) {
			var dx = end.x - start.x; var dy = end.y - start.y;
			return ((point.x - start.x) * dx + (point.y - start.y) * dy) / (dx * dx + dy * dy);
		}
		var c = arcCenter(); var angle = Math.atan2(point.y - c.y, point.x - c.x);
		var delta = positiveAngle(sweep > 0 ? angle - startAngle : startAngle - angle);
		if (delta > 2 * Math.PI - 1e-10) delta = 0;
		return delta / Math.abs(sweep);
	}
	/** Endpoints and cardinal extrema, sufficient for exact projected bounds of this arc. */
	public function extrema():Array<Point2> {
		var result = [start, end];
		if (center != null) {
			var c = arcCenter();
			for (i in 0...4) {
				var angle = i * Math.PI / 2;
				var distance = sweep > 0 ? positiveAngle(angle - startAngle) : positiveAngle(startAngle - angle);
				if (distance <= Math.abs(sweep) + 1e-12)
					result.push(new Point2(c.x + radius * Math.cos(angle), c.y + radius * Math.sin(angle)));
			}
		}
		return result;
	}
	static function positiveAngle(angle:Float):Float {
		var a = angle % (2 * Math.PI);
		return a < 0 ? a + 2 * Math.PI : a;
	}
}

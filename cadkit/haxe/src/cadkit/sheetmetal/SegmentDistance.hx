package cadkit.sheetmetal;

/** Analytic distance and support queries for exact line/arc design checks. */
class SegmentDistance {
	static function onArc(arc:Segment, point:Point2):Bool { return arc.fractionAt(point) <= 1 + 1e-10; }
	public static function point(curve:Segment, point:Point2):Float {
		if (curve.isLine()) {
			var fraction = curve.fractionAt(point);
			return point.distance(curve.point(Math.max(0, Math.min(1, fraction))));
		}
		var result = Math.min(point.distance(curve.start), point.distance(curve.end));
		var center = curve.arcCenter(); var length = center.distance(point);
		if (length > 1e-12) {
			var radial = new Point2(center.x + (point.x - center.x) * curve.radius / length, center.y + (point.y - center.y) * curve.radius / length);
			if (onArc(curve, radial)) result = Math.min(result, point.distance(radial));
		}
		return result;
	}
	public static function toLine(curve:Segment, line:Segment):Float {
		if (!line.isLine()) throw new SheetDiagnosis("sheet.expected-fold-line", "Distance target must be a straight fold line");
		if (OutlineGeometry.intersections(curve, line).length > 0) return 0;
		var result = Math.min(Math.min(point(curve, line.start), point(curve, line.end)), Math.min(point(line, curve.start), point(line, curve.end)));
		if (!curve.isLine()) {
			var center = curve.arcCenter(); var nx = -(line.end.y - line.start.y) / line.length(); var ny = (line.end.x - line.start.x) / line.length();
			for (sign in [-1.0, 1.0]) {
				var radial = new Point2(center.x + sign * nx * curve.radius, center.y + sign * ny * curve.radius);
				if (onArc(curve, radial)) result = Math.min(result, point(line, radial));
			}
		}
		return result;
	}
	public static function support(curve:Segment, direction:Point2):Float {
		var result = Math.max(curve.start.x * direction.x + curve.start.y * direction.y, curve.end.x * direction.x + curve.end.y * direction.y);
		var length = Math.sqrt(direction.x * direction.x + direction.y * direction.y);
		if (length <= 1e-12) throw new SheetDiagnosis("sheet.invalid-direction", "Support direction must be nonzero");
		if (!curve.isLine()) {
			var center = curve.arcCenter(); var radial = new Point2(center.x + curve.radius * direction.x / length, center.y + curve.radius * direction.y / length);
			if (onArc(curve, radial)) result = Math.max(result, radial.x * direction.x + radial.y * direction.y);
		}
		return result;
	}
}

package cadkit.sheetmetal;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/** Minimum outer-profile leg over material-bearing seam intervals; holes have their own clearance rule. */
class FlangeDepth {
	static function coordinate(curve:Segment, y:Float, branch:Float):Float {
		if (curve.isLine()) return curve.start.x + (y - curve.start.y) * (curve.end.x - curve.start.x) / (curve.end.y - curve.start.y);
		var center = curve.arcCenter(); var delta = y - center.y;
		return center.x + branch * Math.sqrt(Math.max(0, curve.radius * curve.radius - delta * delta));
	}
	public static function minimum(outline:WallOutline, attachment:BendAttachment):Float {
		var span = attachment.span; var width = span.length();
		var normal = new Vector(-(span.end.y - span.start.y) / width, (span.end.x - span.start.x) / width);
		var mapped = SheetProfile.transform(outline, Plane.XY(), new Plane(new Vector(span.start.x, span.start.y), normal, Vector.Z()));
		var curves = mapped.loops()[0]; var events:Array<Float> = []; var maximum = 0.0;
		for (curve in curves) for (point in curve.extrema()) { events.push(point.y); maximum = Math.max(maximum, point.x); }
		var result = Math.POSITIVE_INFINITY;
		for (interval in BendSeam.intervals(outline, span, true)) {
			var first = -interval.last * width; var last = -interval.first * width; var values = [first, last];
			for (event in events) if (event > first + 1e-8 && event < last - 1e-8) values.push(event);
			values.sort(function(a:Float, b:Float):Int { return a < b ? -1 : a > b ? 1 : 0; });
			for (i in 0...(values.length - 1)) {
				if (values[i + 1] - values[i] <= 1e-8) continue;
				var y = (values[i] + values[i + 1]) / 2;
				var ray = Segment.line(new Point2(0, y), new Point2(maximum + 1, y));
				var selected:Null<Segment> = null; var nearest = Math.POSITIVE_INFINITY; var branch = 1.0;
				for (curve in curves) for (point in OutlineGeometry.intersections(ray, curve)) if (point.x > 1e-8 && point.x < nearest) {
					selected = curve; nearest = point.x;
					if (!curve.isLine()) branch = point.x < curve.arcCenter().x ? -1 : 1;
				}
				if (selected == null) throw new SheetDiagnosis("sheet.flange-depth", "No outer boundary intersects material from the bend attachment", [attachment.identity]);
				// Vertex and cardinal events split each selected branch into monotone pieces.
				result = Math.min(result, Math.min(coordinate(selected, values[i], branch), coordinate(selected, values[i + 1], branch)));
			}
		}
		if (!Math.isFinite(result)) throw new SheetDiagnosis("sheet.flange-depth", "No measurable material-bearing interval at the bend attachment", [attachment.identity]);
		return result;
	}
}

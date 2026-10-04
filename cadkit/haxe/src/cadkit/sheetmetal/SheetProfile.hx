package cadkit.sheetmetal;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sketch.ProfileCurve;
import cadkit.sketch.ProfileRegion;

/** One solved sketch region mapped exactly into a parallel sheet-wall frame, preserving authored curve identities. */
class SheetProfile {
	public static function region(region:ProfileRegion, source:Plane, target:Plane):WallOutline {
		if (Math.abs(source.normal.dot(target.normal)) < 1 - 1e-9)
			throw new SheetDiagnosis("sheet.cut-plane", "Sketch and wall planes must be parallel");
		var outer = loop(region.outer(), source, target, false);
		var holes = [for (curves in region.holes()) loop(curves, source, target, true)];
		var identities = [outer.identities]; for (hole in holes) identities.push(hole.identities);
		return new WallOutline(outer.segments, [for (hole in holes) hole.segments], identities);
	}
	static function loop(curves:Array<ProfileCurve>, source:Plane, target:Plane, clockwise:Bool):{segments:Array<Segment>, identities:Array<String>} {
		var segments:Array<Segment> = []; var identities:Array<String> = []; var area = 0.0;
		var orientation = source.normal.dot(target.normal) > 0 ? 1 : -1;
		for (curve in curves) {
			var start = point(curve.start, source, target); var end = point(curve.end, source, target);
			var segment:Segment;
			if (curve.center == null) segment = Segment.line(start, end);
			else {
				var center = point(curve.arcCenter(), source, target);
				segment = Segment.arc(center, curve.radius, Math.atan2(start.y - center.y, start.x - center.x), curve.sweep * orientation);
			}
			segments.push(segment); identities.push("sketch:" + curve.entityId); area += segment.area();
		}
		if ((area < 0) != clockwise) {
			var reversed:Array<Segment> = []; var index = segments.length;
			while (index > 0) {
				var segment = segments[--index];
				reversed.push(segment.isLine() ? Segment.line(segment.end, segment.start)
					: Segment.arc(segment.arcCenter(), segment.radius, segment.startAngle + segment.sweep, -segment.sweep));
			}
			identities.reverse(); segments = reversed;
		}
		return {segments: segments, identities: identities};
	}
	static function point(local:Vector, source:Plane, target:Plane):Point2 {
		var transformed = target.toLocal(source.toWorld(local)); return new Point2(transformed.x, transformed.y);
	}
}

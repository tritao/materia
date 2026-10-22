package camkit;

import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfileGeometry;
import materia.casting.CastingProfiles;
import toolpathkit.path.Point3;

/** Neutral finished line/arc profiles -> CAM contours, without compensation.
 * Profile coordinates and XY normalization are mm; tolerance, margin and Z are m.
 * The same conversion accepts panel operations and filled tooling sections.
 */
class CamCastingContours {
	public static function fromProfile(profile:Array<CastingSegment>,tolerance:Float,mirrorX:Bool=false,
			minX:Float=0,minY:Float=0,widthMm:Float=0,margin:Float=0,z:Float=0,
			context:String="Casting CAM"):CamContour {
		if (!Math.isFinite(tolerance) || tolerance <= 0)
			throw context+" needs positive chord tolerance";
		if (!Math.isFinite(z)) throw context+" needs a finite work plane";
		try CastingProfileGeometry.validate(profile) catch (error:Dynamic)
			throw context+": "+Std.string(error);
		var points:Array<Point3> = [];
		var exact:Array<cadkit.sheetmetal.Segment> = [];
		function map(p:materia.casting.CastingPrism.CastingPoint2):cadkit.sheetmetal.Point2 {
			var x = p.x - minX;
			return new cadkit.sheetmetal.Point2((mirrorX ? widthMm - x : x) * 0.001 + margin, (p.y - minY) * 0.001 + margin);
		}
		for (s in profile) {
			var a = map(s.start), b = map(s.end);
			if (s.center == null)
				exact.push(cadkit.sheetmetal.Segment.line(a, b));
			else {
				var center = map(s.center);
				exact.push(cadkit.sheetmetal.Segment.arc(center, CastingProfiles.distance(s.center, s.start) * 0.001,
					Math.atan2(a.y - center.y, a.x - center.x), s.sweep * (mirrorX ? -1 : 1)));
			}
			var count = 1;
			if (s.center != null) {
				var radius = CastingProfiles.distance(s.center, s.start) * 0.001;
				var angle = 2 * Math.acos(Math.max(-1, Math.min(1, 1 - tolerance / radius)));
				if (angle <= 0)
					throw context+" chord tolerance is below numeric resolution";
				count = Std.int(Math.ceil(Math.abs(s.sweep) / Math.min(Math.PI / 2, angle)));
			}
			if (count > 10000 || points.length + count > 50000)
				throw context+" contour exceeds sampling limit";
			for (i in 0...count) {
				var p = map(CastingProfiles.point(s, i / count));
				points.push(new Point3(p.x,p.y,z));
			}
		}
		var result = new CamContour(points);
		result.exactBoundary = exact;
		result.chordTolerance = tolerance;
		return result;
	}
}

package materia.tooling.panel;

import materia.casting.CastingProfileGeometry;

import materia.casting.CastingPrism.CastingPoint2;
import materia.casting.CastingProfile.CastingSegment;
import materia.tooling.panel.PanelPlan.PanelOperation;
import materia.tooling.panel.PanelPlan.PanelOperationKind;

/** Single exact-contour boundary used by CAD, drawings and the future CAM bridge. */
class PanelContours {
	public static function operation(op:PanelOperation):Array<CastingSegment> {
		CastingProfileGeometry.validate(op.contour);
		return [
			for (s in op.contour) {
				var c = s.center;
				{
					kind: s.kind,
					start: {x: s.start.x, y: s.start.y},
					end: {x: s.end.x, y: s.end.y},
					center: c == null ? null : {
						x: c.x,
						y: c.y
					},
					sweep: s.sweep
				};
			}
		];
	}

	public static function outside(panel:PanelPlan):Array<CastingSegment> {
		var found = panel.operations.filter(o -> o.kind == OutsideProfile);
		if (found.length != 1)
			throw "Panel needs exactly one outside-profile operation";
		return operation(found[0]);
	}

	public static function line(a:CastingPoint2, b:CastingPoint2):CastingSegment
		return materia.casting.CastingProfiles.line(a,b);
	public static function arc(x:Float,y:Float,r:Float,start:Float,sweep:Float):CastingSegment
		return materia.casting.CastingProfiles.arc(x,y,r,start,sweep);
	public static function point(s:CastingSegment,t:Float):CastingPoint2
		return materia.casting.CastingProfiles.point(s,t);
	public static function distance(a:CastingPoint2,b:CastingPoint2):Float
		return materia.casting.CastingProfiles.distance(a,b);
	public static function polygon(points:Array<CastingPoint2>):Array<CastingSegment>
		return materia.casting.CastingProfiles.polygon(points);
	public static function rectangle(x:Float,y:Float,w:Float,h:Float):Array<CastingSegment>
		return materia.casting.CastingProfiles.rectangle(x,y,w,h);
	public static function circle(x:Float,y:Float,r:Float):Array<CastingSegment>
		return materia.casting.CastingProfiles.circle(x,y,r);
	public static function obround(x:Float,y:Float,w:Float,h:Float):Array<CastingSegment>
		return materia.casting.CastingProfiles.obround(x,y,w,h);
	public static function roundedRectangle(x:Float,y:Float,w:Float,h:Float,r:Float):Array<CastingSegment>
		return materia.casting.CastingProfiles.roundedRectangle(x,y,w,h,r);
	public static function placed(contour:Array<CastingSegment>,radians:Float,x:Float,y:Float):Array<CastingSegment>
		return materia.casting.CastingProfiles.placed(contour,radians,x,y);

	/** Exact boundary of a rectangle union four corner circles; no overlapping loops. */
	public static function relievedRectangle(x:Float, y:Float, w:Float, h:Float, r:Float):Array<CastingSegment> {
		if (r <= 0 || w <= 2 * r || h <= 2 * r)
			throw "Relief tool does not fit rectangle";
		var l = x - w / 2, b = y - h / 2, rr = x + w / 2, t = y + h / 2;
		return [
			line({x: l + r, y: b}, {x: rr - r, y: b}),
			arc(rr, b, r, Math.PI, 3 * Math.PI / 2),
			line({x: rr, y: b + r}, {x: rr, y: t - r}),
			arc(rr, t, r, -Math.PI / 2, 3 * Math.PI / 2),
			line({x: rr - r, y: t}, {x: l + r, y: t}),
			arc(l, t, r, 0, 3 * Math.PI / 2),
			line({x: l, y: t - r}, {x: l, y: b + r}),
			arc(l, b, r, Math.PI / 2, 3 * Math.PI / 2)
		];
	}

	/** Integrate circular overcuts at concave right-angle roots into the outside loop. */
	public static function relievedPolygon(p:Array<CastingPoint2>, r:Float, roundConvex:Bool = false):Array<CastingSegment> {
		var starts:Array<CastingPoint2> = [],
			ends:Array<CastingPoint2> = [],
			corners:Array<Null<CastingSegment>> = [];
		for (i in 0...p.length) {
			var a = p[(i + p.length - 1) % p.length], v = p[i], b = p[(i + 1) % p.length];
			var da = distance(a, v),
				db = distance(v, b),
				ux = (v.x - a.x) / da,
				uy = (v.y - a.y) / da,
				wx = (b.x - v.x) / db,
				wy = (b.y - v.y) / db;
			if (ux * wy - uy * wx < -0.5) {
				if (da <= 2 * r || db <= 2 * r)
					throw "Insufficient space for root relief";
				var s = {x: v.x - ux * r, y: v.y - uy * r},
					e = {x: v.x + wx * r, y: v.y + wy * r};
				starts.push(s);
				ends.push(e);
				corners.push(arc(v.x, v.y, r, Math.atan2(s.y - v.y, s.x - v.x), -3 * Math.PI / 2));
			} else if (roundConvex && ux * wy - uy * wx > 0.5) {
				if (da <= 2 * r || db <= 2 * r)
					throw "Insufficient space for rounded corner";
				var start = {x: v.x - ux * r, y: v.y - uy * r},
					end = {x: v.x + wx * r, y: v.y + wy * r};
				var center = {x: start.x + wx * r, y: start.y + wy * r};
				starts.push(start);
				ends.push(end);
				corners.push(arc(center.x, center.y, r, Math.atan2(start.y - center.y, start.x - center.x), Math.PI / 2));
			} else {
				starts.push(v);
				ends.push(v);
				corners.push(null);
			}
		}
		var result:Array<CastingSegment> = [];
		for (i in 0...p.length) {
			var corner = corners[i];
			if (corner != null)
				result.push(corner);
			result.push(line(ends[i], starts[(i + 1) % p.length]));
		}
		return result;
	}
}

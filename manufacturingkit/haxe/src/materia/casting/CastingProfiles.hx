package materia.casting;

import materia.casting.CastingPrism.CastingPoint2;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfile.CastingCurveKind;

/** Exact closed profiles, without cutter compensation or tessellation. */
class CastingProfiles {
	public static function line(a:CastingPoint2, b:CastingPoint2):CastingSegment
		return {
			kind: CastingCurveKind.Line,
			start: {x: a.x, y: a.y},
			end: {
				x: b.x,
				y: b.y
			},
			center: null,
			sweep: 0
		};

	public static function arc(x:Float, y:Float, r:Float, start:Float, sweep:Float):CastingSegment {
		if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(r) || r <= 0 || !Math.isFinite(start) || !Math.isFinite(sweep))
			throw "Invalid arc parameters";
		return {
			kind: CastingCurveKind.Arc,
			start: {x: x + r * Math.cos(start), y: y + r * Math.sin(start)},
			end: {
				x: x + r * Math.cos(start + sweep),
				y: y + r * Math.sin(start + sweep)
			},
			center: {
				x: x,
				y: y
			},
			sweep: sweep
		};
	}

	public static function point(s:CastingSegment, t:Float):CastingPoint2 {
		var c = s.center;
		if (c == null)
			return {x: s.start.x + (s.end.x - s.start.x) * t, y: s.start.y + (s.end.y - s.start.y) * t};
		var r = distance(c, s.start),
			a = Math.atan2(s.start.y - c.y, s.start.x - c.x) + s.sweep * t;
		return {x: c.x + r * Math.cos(a), y: c.y + r * Math.sin(a)};
	}

	public static function distance(a:CastingPoint2, b:CastingPoint2):Float
		return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

	public static function polygon(points:Array<CastingPoint2>):Array<CastingSegment>
		return [
			for (i in 0...points.length)
				line(points[i], points[(i + 1) % points.length])
		];

	public static function rectangle(x:Float, y:Float, w:Float, h:Float):Array<CastingSegment> {
		if (!Math.isFinite(w) || !Math.isFinite(h) || w <= 0 || h <= 0)
			throw "Invalid rectangle dimensions";
		return polygon([{x: x, y: y}, {x: x + w, y: y}, {x: x + w, y: y + h}, {x: x, y: y + h}]);
	}

	public static function circle(x:Float, y:Float, r:Float):Array<CastingSegment>
		return [arc(x, y, r, 0, Math.PI), arc(x, y, r, Math.PI, Math.PI)];

	public static function obround(x:Float, y:Float, w:Float, h:Float):Array<CastingSegment> {
		if (!Math.isFinite(w + h) || w <= 0 || h <= 0)
			throw "Invalid obround";
		if (w < h)
			return [for (segment in obround(y, -x, h, w)) {
				kind: segment.kind,
				start: {x: -segment.start.y, y: segment.start.x},
				end: {x: -segment.end.y, y: segment.end.x},
				center: segment.center == null ? null : {x: -segment.center.y, y: segment.center.x},
				sweep: segment.sweep
			}];
		if (w == h)
			return circle(x, y, h / 2);
		var a = (w - h) / 2, r = h / 2;
		return [
			line({x: x - a, y: y - r}, {x: x + a, y: y - r}),
			arc(x + a, y, r, -Math.PI / 2, Math.PI),
			line({x: x + a, y: y + r}, {x: x - a, y: y + r}),
			arc(x - a, y, r, Math.PI / 2, Math.PI)
		];
	}

	public static function roundedRectangle(x:Float, y:Float, w:Float, h:Float, r:Float):Array<CastingSegment> {
		if (r <= 0 || w <= 2 * r || h <= 2 * r)
			throw "Corner radius does not fit rectangle";
		var right = x + w, top = y + h;
		return [
			line({x: x + r, y: y}, {x: right - r, y: y}),
			arc(right - r, y + r, r, -Math.PI / 2, Math.PI / 2),
			line({x: right, y: y + r}, {x: right, y: top - r}),
			arc(right - r, top - r, r, 0, Math.PI / 2),
			line({x: right - r, y: top}, {x: x + r, y: top}),
			arc(x + r, top - r, r, Math.PI / 2, Math.PI / 2),
			line({x: x, y: top - r}, {x: x, y: y + r}),
			arc(x + r, y + r, r, Math.PI, Math.PI / 2)
		];
	}

	/** Rigid 2D placement preserves exact arcs, including their signed sweep. */
	public static function placed(contour:Array<CastingSegment>, radians:Float, x:Float, y:Float):Array<CastingSegment> {
		var c = Math.cos(radians), s = Math.sin(radians);
		function move(p:CastingPoint2):CastingPoint2
			return {x: x + c * p.x - s * p.y, y: y + s * p.x + c * p.y};
		return [
			for (seg in contour) {
				var center = seg.center;
				{
					kind: seg.kind,
					start: move(seg.start),
					end: move(seg.end),
					center: center == null ? null : move(center),
					sweep: seg.sweep
				};
			}
		];
	}

	/** Circular fillets of a line polygon, including re-entrant corners. Adjacent
	 * tangency distances must fit their common edge; overlapping fillets fail.
	 * The exact arcs returned here are shared by CAD, drawings and CAM.
	 */
	public static function roundedPolygon(points:Array<CastingPoint2>,radius:Float):Array<CastingSegment> {
		CastingProfileGeometry.validate(polygon(points));
		if (!Math.isFinite(radius) || radius<=0) throw "Invalid polygon corner radius";
		var starts:Array<CastingPoint2> = [],ends:Array<CastingPoint2> = [];
		var corners:Array<Null<CastingSegment>> = [];
		var trims:Array<Float> = [];
		for (index in 0...points.length) {
			var a=points[(index+points.length-1)%points.length],v=points[index],b=points[(index+1)%points.length];
			var incoming=distance(a,v),outgoing=distance(v,b);
			var ux=(v.x-a.x)/incoming,uy=(v.y-a.y)/incoming,wx=(b.x-v.x)/outgoing,wy=(b.y-v.y)/outgoing;
			var turn=Math.atan2(ux*wy-uy*wx,ux*wx+uy*wy);
			var trim=radius*Math.tan(Math.abs(turn)/2);
			if (!Math.isFinite(trim)) throw "Corner radius does not fit polygon";
			trims.push(trim);
			if (Math.abs(turn)<1e-10) {
				starts.push(v);ends.push(v);corners.push(null);
				continue;
			}
			var start={x:v.x-ux*trim,y:v.y-uy*trim},end={x:v.x+wx*trim,y:v.y+wy*trim};
			var sign=turn>0 ? 1.0 : -1.0,center={x:start.x-uy*radius*sign,y:start.y+ux*radius*sign};
			starts.push(start);ends.push(end);
			corners.push(arc(center.x,center.y,radius,Math.atan2(start.y-center.y,start.x-center.x),turn));
		}
		var result:Array<CastingSegment> = [];
		for (index in 0...points.length) {
			var next=(index+1)%points.length,length=distance(points[index],points[next]);
			var remaining=length-trims[index]-trims[next];
			if (remaining< -1e-9) throw "Corner radius does not fit polygon";
			if (corners[index]!=null) result.push(corners[index]);
			if (remaining>1e-9) result.push(line(ends[index],starts[next]));
		}
		CastingProfileGeometry.validate(result);
		return result;
	}

}

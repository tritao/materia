package materia.tooling.panel;

import materia.tooling.ToolingPrism.ToolingPoint2;
import materia.tooling.panel.PanelPlan.PanelSegment;
import materia.tooling.panel.PanelPlan.PanelCurveKind;
import materia.tooling.panel.PanelPlan.PanelOperation;
import materia.tooling.panel.PanelPlan.PanelOperationKind;

/** Single exact-contour boundary used by CAD, drawings and the future CAM bridge. */
class PanelContours {
	public static function operation(op:PanelOperation):Array<PanelSegment> {
		PanelGeometry.validate(op.contour);
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

	public static function outside(panel:PanelPlan):Array<PanelSegment> {
		var found = panel.operations.filter(o -> o.kind == OutsideProfile);
		if (found.length != 1)
			throw "Panel needs exactly one outside-profile operation";
		return operation(found[0]);
	}

	public static function line(a:ToolingPoint2, b:ToolingPoint2):PanelSegment
		return {
			kind: Line,
			start: {x: a.x, y: a.y},
			end: {
				x: b.x,
				y: b.y
			},
			center: null,
			sweep: 0
		};

	public static function arc(x:Float, y:Float, r:Float, start:Float, sweep:Float):PanelSegment {
		if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(r) || r <= 0 || !Math.isFinite(start) || !Math.isFinite(sweep))
			throw "Invalid arc parameters";
		return {
			kind: Arc,
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

	public static function point(s:PanelSegment, t:Float):ToolingPoint2 {
		var c = s.center;
		if (c == null)
			return {x: s.start.x + (s.end.x - s.start.x) * t, y: s.start.y + (s.end.y - s.start.y) * t};
		var r = distance(c, s.start),
			a = Math.atan2(s.start.y - c.y, s.start.x - c.x) + s.sweep * t;
		return {x: c.x + r * Math.cos(a), y: c.y + r * Math.sin(a)};
	}

	public static function distance(a:ToolingPoint2, b:ToolingPoint2):Float
		return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

	public static function polygon(points:Array<ToolingPoint2>):Array<PanelSegment>
		return [
			for (i in 0...points.length)
				line(points[i], points[(i + 1) % points.length])
		];

	public static function rectangle(x:Float, y:Float, w:Float, h:Float):Array<PanelSegment> {
		if (!Math.isFinite(w) || !Math.isFinite(h) || w <= 0 || h <= 0)
			throw "Invalid rectangle dimensions";
		return polygon([{x: x, y: y}, {x: x + w, y: y}, {x: x + w, y: y + h}, {x: x, y: y + h}]);
	}

	public static function circle(x:Float, y:Float, r:Float):Array<PanelSegment>
		return [arc(x, y, r, 0, Math.PI), arc(x, y, r, Math.PI, Math.PI)];

	public static function obround(x:Float, y:Float, w:Float, h:Float):Array<PanelSegment> {
		if (w < h || h <= 0)
			throw "Invalid obround";
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

	public static function roundedRectangle(x:Float, y:Float, w:Float, h:Float, r:Float):Array<PanelSegment> {
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
	public static function placed(contour:Array<PanelSegment>, radians:Float, x:Float, y:Float):Array<PanelSegment> {
		var c = Math.cos(radians), s = Math.sin(radians);
		function move(p:ToolingPoint2):ToolingPoint2
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

	/** Exact boundary of a rectangle union four corner circles; no overlapping loops. */
	public static function relievedRectangle(x:Float, y:Float, w:Float, h:Float, r:Float):Array<PanelSegment> {
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
	public static function relievedPolygon(p:Array<ToolingPoint2>, r:Float, roundConvex:Bool = false):Array<PanelSegment> {
		var starts:Array<ToolingPoint2> = [],
			ends:Array<ToolingPoint2> = [],
			corners:Array<Null<PanelSegment>> = [];
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
		var result:Array<PanelSegment> = [];
		for (i in 0...p.length) {
			var corner = corners[i];
			if (corner != null)
				result.push(corner);
			result.push(line(ends[i], starts[(i + 1) % p.length]));
		}
		return result;
	}
}

package cadkit.sheetmetal;

import cadkit.modeling.Vector;

/** Certified coordinate enclosures of exact mapped boundary curves, within 1e-8 mm. */
class BendMaterialBounds {
	public static function include(result:SheetBounds, zone:BendZone, regions:Array<WallOutline>):Void {
		var lower = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var upper = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (region in regions) for (segment in region.loops()[0]) for (depth in [0.0, zone.thickness]) for (axis in 0...3) {
			var extrema = new BendCurveBounds(zone, segment, depth, axis);
			lower[axis] = Math.min(lower[axis], extrema.lower()); upper[axis] = Math.max(upper[axis], extrema.upper());
		}
		result.include(new Vector(lower[0], lower[1], lower[2]));
		result.include(new Vector(upper[0], upper[1], upper[2]));
	}
}

/** Linear interpolation error is bounded by max |f''| * interval² / 8; no profile tessellation. */
private class BendCurveBounds {
	static inline var EPS:Float = 1e-8;
	final zone:BendZone;
	final segment:Segment;
	final depth:Float;
	final axis:Int;
	final curvature:Float;
	var minimum:Float;
	var maximum:Float;
	public function new(zone:BendZone, segment:Segment, depth:Float, axis:Int) {
		this.zone = zone; this.segment = segment; this.depth = depth; this.axis = axis;
		var x = Math.abs(component(zone.frame.plane.xDirection));
		var y = Math.abs(component(zone.frame.plane.yDirection));
		var z = Math.abs(component(zone.frame.plane.normal));
		var dx = segment.isLine() ? Math.abs(segment.end.x - segment.start.x) : segment.radius * Math.abs(segment.sweep);
		var second = segment.isLine() ? 0.0 : segment.radius * segment.sweep * segment.sweep;
		if (zone.angle == 0) curvature = (x + y) * second;
		else {
			var radius = zone.curvedInnerRadius() + (zone.angle > 0 ? depth : zone.thickness - depth);
			var frequency = Math.abs(zone.angle) / zone.allowance;
			curvature = radius * (x + z) * (frequency * frequency * dx * dx + frequency * second) + y * second;
		}
		var first = value(0); var last = value(1); minimum = Math.min(first, last); maximum = Math.max(first, last);
		visit(0, 1, first, last, 0);
	}
	function component(vector:Vector):Float { return axis == 0 ? vector.x : axis == 1 ? vector.y : vector.z; }
	function value(fraction:Float):Float {
		var point = segment.point(fraction);
		if (point.x < -1e-8 || point.x > zone.allowance + 1e-8 || point.y < -1e-8 || point.y > zone.width + 1e-8)
			throw new SheetDiagnosis("sheet.bend-material-domain", "Developed bend boundary lies outside its nominal strip", ["bend:" + zone.id]);
		return component(zone.point(Math.max(0, Math.min(1, point.x / zone.allowance)), Math.max(0, Math.min(zone.width, point.y)), depth));
	}
	function visit(first:Float, last:Float, a:Float, b:Float, level:Int):Void {
		var midpoint = (first + last) / 2; var center = value(midpoint);
		minimum = Math.min(minimum, center); maximum = Math.max(maximum, center);
		var error = curvature * (last - first) * (last - first) / 8;
		if (Math.min(a, b) - error >= minimum - EPS && Math.max(a, b) + error <= maximum + EPS) return;
		if (level >= 50) throw new SheetDiagnosis("sheet.bend-bounds-precision", "Could not bound the mapped bend curve at requested precision", ["bend:" + zone.id]);
		visit(first, midpoint, a, center, level + 1); visit(midpoint, last, center, b, level + 1);
	}
	public function lower():Float { return minimum - EPS; }
	public function upper():Float { return maximum + EPS; }
}

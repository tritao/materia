package cadkit.sheetmetal;

import cadkit.modeling.Location;
import cadkit.modeling.Vector;

/** A fresh, native-free evaluation of wall placements and bend strips. */
class SheetEvaluation {
	final frames:Map<Int, Location>;
	final zones:Map<Int, BendZone>;
	final walls:Array<Wall>;
	final bends:Array<Bend>;
	final thickness:Float;
	public function new(frames:Map<Int, Location>, zones:Map<Int, BendZone>, walls:Array<Wall>, bends:Array<Bend>, thickness:Float) {
		this.frames = frames;
		this.zones = zones;
		this.walls = walls.copy();
		this.bends = bends.copy();
		this.thickness = thickness;
	}
	public function wall(id:Int):Location {
		var result = frames.get(id);
		if (result == null)
			throw new SheetDiagnosis("sheet.missing-wall", "Wall has no evaluated frame", ["wall:" + id]);
		return result;
	}
	public function bend(id:Int):BendZone {
		var result = zones.get(id);
		if (result == null)
			throw new SheetDiagnosis("sheet.missing-bend", "Bend has no evaluated strip", ["bend:" + id]);
		return result;
	}

	/** Analytic prism/strip extrema; trimmed mapped curves are enclosed within 1e-8 mm. */
	public function bounds():SheetBounds {
		var result = new SheetBounds();
		for (w in walls) {
			var placement = wall(w.id);
			for (loop in w.outline.loops())
				for (segment in loop) {
					var fractions = [0.0, 1.0];
					if (!segment.isLine()) {
						var x = placement.plane.xDirection;
						var y = placement.plane.yDirection;
						for (a in [Math.atan2(y.x, x.x), Math.atan2(y.y, x.y), Math.atan2(y.z, x.z)]) {
							addArcFraction(fractions, a, segment.startAngle, segment.sweep);
							addArcFraction(fractions, a + Math.PI, segment.startAngle, segment.sweep);
						}
					}
					for (fraction in fractions) {
						var p = segment.point(fraction);
						for (z in [0.0, -thickness])
							result.include(placement.plane.toWorld(new Vector(p.x, p.y, z)));
					}
				}
		}
		for (record in bends) {
			var zone = bend(record.id);
			if (record.hasCuts()) { BendMaterialBounds.include(result, zone, record.materialRegions(zone.allowance)); continue; }
			var fractions = [0.0, 1.0];
			if (zone.angle != 0) {
				var x = zone.frame.plane.xDirection;
				var z = zone.frame.plane.normal;
				var sign = zone.angle > 0 ? 1.0 : -1.0;
				for (a in [Math.atan2(-x.x, sign * z.x), Math.atan2(-x.y, sign * z.y), Math.atan2(-x.z, sign * z.z)])
					for (i in -2...3) {
						var candidate = a + i * Math.PI;
						if (candidate > 0 && candidate < Math.abs(zone.angle))
							fractions.push(candidate / Math.abs(zone.angle));
					}
			}
			for (f in fractions)
				for (y in [0.0, zone.width])
					for (depth in [0.0, zone.thickness])
						result.include(zone.point(f, y, depth));
		}
		return result;
	}
	static function addArcFraction(fractions:Array<Float>, angle:Float, start:Float, sweep:Float):Void {
		var d = (sweep > 0 ? angle - start : start - angle) % (2 * Math.PI);
		if (d < 0)
			d += 2 * Math.PI;
		if (d <= Math.abs(sweep))
			fractions.push(d / Math.abs(sweep));
	}
}

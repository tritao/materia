package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.Bend;
import cadkit.sheetmetal.BendZone;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.WallOutline;

/** Evaluated bend material as convex pieces, with explicit curvature and cut-mapping error in mm. */
class BendPrismProfile {
	public final error:Float;
	final solids:Array<ConvexPrism>;
	function new(error:Float, solids:Array<ConvexPrism>) { this.error = error; this.solids = solids.copy(); }
	public function pieces():Array<ConvexPrism> return solids.copy();

	/** Canonical developed cut regions remain the material owner through flat and curved states. */
	public static function of(record:Bend, zone:BendZone, maximumError:Float = 0.01):BendPrismProfile {
		if (record.id != zone.id || Math.abs(record.edgeA.span.length() - zone.width) > 1e-8)
			throw new SheetDiagnosis("press.bend-profile", "Bend material and evaluated zone identities or widths disagree", ["bend:" + record.id]);
		if (!Math.isFinite(maximumError) || maximumError <= 0)
			throw new SheetDiagnosis("press.bend-profile", "Bend approximation error must be finite positive mm", ["bend:" + record.id]);
		var id = "bend:" + record.id, result:Array<ConvexPrism> = [], error = 0.0;
		var regions = record.materialRegions(zone.allowance);
		if (zone.angle == 0) {
			for (i in 0...regions.length) {
				var profile = SheetPrismProfile.of(id + ":region:" + i, regions[i], zone.thickness, maximumError);
				error = Math.max(error, profile.curveError);
				for (piece in profile.pieces()) result.push(piece.placed(zone.frame));
			}
			return new BendPrismProfile(error, result);
		}
		var radius = zone.curvedInnerRadius() + zone.thickness;
		var step = Math.min(Math.PI / 2, Math.sqrt(8 * maximumError / radius));
		if (record.hasCuts()) step = Math.min(step / Math.sqrt(3), 2 * maximumError / (3 * zone.thickness));
		var estimate = Math.abs(zone.angle) / step;
		if (!Math.isFinite(estimate) || estimate > 1000000)
			throw new SheetDiagnosis("press.bend-profile-resolution", "Requested error exceeds the one-million-sector bend budget", [id]);
		var count = Math.ceil(estimate); if (count < 1) count = 1;
		var sign = zone.angle > 0 ? 1.0 : -1.0;
		var innerDepth = sign > 0 ? 0.0 : zone.thickness;
		var outerDepth = sign > 0 ? zone.thickness : 0.0;
		for (i in 0...count) {
			var first = i / count, last = (i + 1) / count;
			var delta = Math.abs(zone.angle) / count;
			if (!record.hasCuts()) {
				var polygon = clean([zone.point(first, 0, innerDepth), zone.point(last, 0, innerDepth),
					zone.point(last, 0, outerDepth), zone.point(first, 0, outerDepth)]);
				result.push(new ConvexPrism(id + ":sector:" + i, polygon, zone.frame.direction(new Vector(0, zone.width, 0))));
				error = Math.max(error, radius * delta * delta / 8);
			} else {
				var left = first * zone.allowance, right = last * zone.allowance;
				var clip = WallOutline.polygon([new Point2(left, 0), new Point2(right, 0),
					new Point2(right, zone.width), new Point2(left, zone.width)]);
				var scale = Math.max(1, radius * Math.abs(zone.angle) / zone.allowance);
				for (region in regions) for (part in OutlineBoolean.intersection(region, clip, id + ":sector:" + i)) {
					var profile = SheetPrismProfile.of(id + ":cut:" + i, part, 1, maximumError / (3 * scale));
					for (piece in profile.pieces()) {
						var vertices = piece.vertices(), size = Std.int(vertices.length / 2);
						for (j in 1...(size - 1)) {
							var source = [vertices[0], vertices[j], vertices[j + 1]];
							var low = source[0].x, high = source[0].x;
							for (point in source) { low = Math.min(low, point.x); high = Math.max(high, point.x); }
							var midpoint = (low + high) / (2 * zone.allowance), angle = zone.angle * midpoint;
							var thickness = zone.frame.direction(new Vector(-Math.sin(angle), 0, Math.cos(angle))).scale(sign * zone.thickness);
							var triangle = [for (point in source) zone.point(point.x / zone.allowance, point.y, outerDepth)];
							result.push(new ConvexPrism(id + ":cut-piece:" + result.length, triangle, thickness));
							var span = Math.abs(zone.angle) * (high - low) / zone.allowance;
							error = Math.max(error, profile.curveError * scale + radius * span * span / 8 + zone.thickness * span / 2);
						}
					}
				}
			}
		}
		return new BendPrismProfile(error, result);
	}
	static function clean(points:Array<Vector>):Array<Vector> {
		var result:Array<Vector> = [];
		for (point in points) if (result.length == 0 || result[result.length - 1].subtract(point).length() > 1e-10) result.push(point);
		if (result.length > 1 && result[0].subtract(result[result.length - 1]).length() <= 1e-10) { var removed = result.pop(); }
		return result;
	}
}

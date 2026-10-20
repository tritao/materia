package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingCavities.FinishedCastingCavity;
import materia.casting.CastingPrism.CastingPoint2;

/** Exact planar exit-disk/material intersection, with contact-only faces excluded.
 * Sections use the authoritative finished cavities; no design cut is clipped here.
 */
class CastingHoleExits {
	public static function intersectsMaterial(body:Array<CastingPrism>, cavities:Array<FinishedCastingCavity>,
			features:Array<CastingFeature>, exit:AssemblyFrame, radius:Float, owner:String):Bool {
		var material:Array<Array<CastingPoint2>> = [];
		for (prism in body) {
			var polygon = section(prism, exit);
			if (polygon.length >= 3) material.push(polygon);
		}
		if (material.length == 0) return false;
		for (cavity in cavities) {
			var source = features.filter(feature -> feature.id == cavity.featureId)[0];
			if (source.owner == owner || source.openings.length == 0) continue;
			for (prism in materia.casting.CastingCavities.prisms(cavity)) {
				var opening = section(prism, exit);
				if (opening.length < 3) continue;
				var next:Array<Array<CastingPoint2>> = [];
				for (polygon in material)
					for (piece in subtract(polygon, opening)) next.push(piece);
				material = next;
			}
		}
		for (polygon in material)
			if (meetsDisk(polygon, radius)) return true;
		return false;
	}

	/** Limit section on the outgoing side: a solid ending at this plane is contact. */
	static function section(prism:CastingPrism, exit:AssemblyFrame):Array<CastingPoint2> {
		var transform = F.compose(F.inverse(exit), prism.pose);
		var low = [for (p in prism.profile) F.transformPoint(transform, p.x, p.y, 0)];
		var high = [for (p in prism.profile) F.transformPoint(transform, p.x, p.y, prism.height)];
		var maximum = Math.NEGATIVE_INFINITY, minimum = Math.POSITIVE_INFINITY;
		for (p in low.concat(high)) { maximum = Math.max(maximum, p.z); minimum = Math.min(minimum, p.z); }
		if (maximum <= 1e-9 || minimum > 1e-9) return [];
		var points:Array<CastingPoint2> = [];
		for (i in 0...low.length) {
			var j = (i + 1) % low.length;
			for (edge in [[low[i], low[j]], [high[i], high[j]], [low[i], high[i]]]) {
				var a = edge[0], b = edge[1];
				if (Math.abs(a.z) <= 1e-9) points.push({x: a.x, y: a.y});
				if (a.z * b.z < 0) {
					var t = a.z / (a.z - b.z);
					points.push({x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y)});
				}
			}
		}
		if (points.length < 3) return [];
		var x = 0.0, y = 0.0;
		for (p in points) { x += p.x; y += p.y; }
		x /= points.length; y /= points.length;
		points.sort((a, b) -> Reflect.compare(Math.atan2(a.y - y, a.x - x), Math.atan2(b.y - y, b.x - x)));
		var clean = CastingPolygons.clean(points);
		return clean.length >= 3 && CastingPolygons.area(clean) > 1e-9 ? clean : [];
	}

	static function subtract(polygon:Array<CastingPoint2>, opening:Array<CastingPoint2>):Array<Array<CastingPoint2>> {
		var result:Array<Array<CastingPoint2>> = [], inside = polygon;
		for (i in 0...opening.length) {
			var p = opening[i], q = opening[(i + 1) % opening.length];
			var a = q.y - p.y, b = p.x - q.x, c = a * p.x + b * p.y;
			var outside = CastingPolygons.halfPlane(inside, -a, -b, -c);
			if (outside.length >= 3 && CastingPolygons.area(outside) > 1e-9) result.push(outside);
			inside = CastingPolygons.halfPlane(inside, a, b, c);
			if (inside.length < 3) break;
		}
		return result;
	}

	static function meetsDisk(polygon:Array<CastingPoint2>, radius:Float):Bool {
		var inside = true, nearest = Math.POSITIVE_INFINITY;
		for (i in 0...polygon.length) {
			var a = polygon[i], b = polygon[(i + 1) % polygon.length];
			var dx = b.x - a.x, dy = b.y - a.y;
			if (dx * -a.y - dy * -a.x < -1e-9) inside = false;
			var t = Math.max(0, Math.min(1, -(a.x * dx + a.y * dy) / (dx * dx + dy * dy)));
			var x = a.x + t * dx, y = a.y + t * dy;
			nearest = Math.min(nearest, x * x + y * y);
		}
		return inside || nearest < radius * radius - 1e-9;
	}
}

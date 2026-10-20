package materia.casting;

import materia.casting.CastingFeature;
import materia.assembly.AssemblyFrames as F;

/** Exact bounds of axis-permuted box features. General profiles are not approximated. */
class CastingBounds {
	public static function axis(direction:{x:Float, y:Float, z:Float}):Null<Int> {
		var values = [direction.x, direction.y, direction.z];
		for (index in 0...3) {
			if (Math.abs(Math.abs(values[index]) - 1) > 1e-8)
				continue;
			var aligned = true;
			for (other in 0...3)
				if (other != index && Math.abs(values[other]) > 1e-8)
					aligned = false;
			if (aligned)
				return index;
		}
		return null;
	}

	public static function feature(feature:CastingFeature):Box3 {
		try return box(feature.geometry)
		catch (error:Dynamic) throw "Unsupported casting feature: " + feature.id + ": " + Std.string(error);
	}

	public static function box(g:CastingGeometry):Box3 {
		var dimensions = CastingFeature.CastingGeometrySolids.box(g.solid, "casting geometry");
		var low = [g.pose.x, g.pose.y, g.pose.z], high = low.copy();
		var localLow = [-dimensions.width / 2, -dimensions.depth / 2, 0.0];
		var localHigh = [dimensions.width / 2, dimensions.depth / 2, dimensions.height];
		var directions = [F.transformVector(g.pose, 1, 0, 0), F.transformVector(g.pose, 0, 1, 0), F.transformVector(g.pose, 0, 0, 1)];
		var used = new Map<Int, Bool>();
		for (local in 0...3) {
			var direction = directions[local], world = axis(direction);
			if (world == null || used.exists(world))
				throw "Non-axis-aligned casting box";
			used.set(world, true);
			var components = [direction.x, direction.y, direction.z], positive = components[world] > 0;
			// Axis-permuted geometry uses exact signed extents. Transforming all eight
			// corners adds quaternion roundoff and can invent weak section slivers.
			low[world] += positive ? localLow[local] : -localHigh[local];
			high[world] += positive ? localHigh[local] : -localLow[local];
		}
		return {x: low[0], y: low[1], z: low[2], width: high[0] - low[0], depth: high[1] - low[1], height: high[2] - low[2]};
	}

}

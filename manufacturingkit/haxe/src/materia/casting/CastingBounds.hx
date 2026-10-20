package materia.casting;

import materia.casting.CastingFeature;
import materia.assembly.AssemblyFrames as F;

typedef CastingBounds3 = {var x:Float; var y:Float; var z:Float; var width:Float; var depth:Float; var height:Float;}

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

	public static function feature(feature:CastingFeature):CastingBounds3 {
		try return box(feature.geometry)
		catch (error:Dynamic) throw "Unsupported casting feature: " + feature.id + ": " + Std.string(error);
	}

	public static function box(g:CastingGeometry):CastingBounds3 {
		var points = [
			for (x in [-g.width / 2, g.width / 2]) for (y in [-g.depth / 2, g.depth / 2]) for (z in [0.0, g.height]) F.transformPoint(g.pose, x, y, z)
		];
		var xs = [for (p in points) p.x],
			ys = [for (p in points) p.y],
			zs = [for (p in points) p.z];
		xs.sort(Reflect.compare);
		ys.sort(Reflect.compare);
		zs.sort(Reflect.compare);
		for (a in [
			F.transformVector(g.pose, 1, 0, 0),
			F.transformVector(g.pose, 0, 1, 0),
			F.transformVector(g.pose, 0, 0, 1)
		])
			if (axis(a) == null)
				throw "Non-axis-aligned casting box";
		return {
			x: xs[0],
			y: ys[0],
			z: zs[0],
			width: xs[7] - xs[0],
			depth: ys[7] - ys[0],
			height: zs[7] - zs[0]
		};
	}

}

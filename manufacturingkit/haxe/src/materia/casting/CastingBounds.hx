package materia.casting;

import materia.casting.CastingFeature;
import materia.assembly.AssemblyFrames as F;

typedef CastingBounds3 = {var x:Float; var y:Float; var z:Float; var width:Float; var depth:Float; var height:Float;}

/** Exact bounds of axis-permuted box features. General profiles are not approximated. */
class CastingBounds {
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
			if (Math.max(Math.abs(a.x), Math.max(Math.abs(a.y), Math.abs(a.z))) < 1 - 1e-8)
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

	public static function openings(b:CastingBounds3, body:CastingBounds3):Array<CastingOpening> {
		var result:Array<CastingOpening> = [];
		for (pair in [
			{key: "x-min", a: b.x, v: body.x}, {key: "x-max", a: b.x + b.width, v: body.x + body.width},
			{key: "y-min", a: b.y, v: body.y}, {key: "y-max", a: b.y + b.depth, v: body.y + body.depth},
			{key: "z-min", a: b.z, v: body.z}, {key: "z-max", a: b.z + b.height, v: body.z + body.height}
		])
			if (Math.abs(pair.a - pair.v) < 1e-7)
				result.push(CastingPatterns.face("body." + pair.key));
		return result;
	}
}

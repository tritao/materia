package materia.casting;

import materia.casting.CastingPrism;
import materia.assembly.AssemblyFrames as F;

/** Disjoint convex envelope pieces for the supported planar body family. */
class CastingEnvelope {
	public static function build(width:Float, depth:Float, height:Float, chamfer:Float = 0, footWidth:Float = 0, footHeight:Float = 0,
			footShoulder:Float = 0):Array<CastingPrism> {
		var x = width / 2, y = depth / 2, c = chamfer;
		var profile = c == 0 ? [{x: -x, y: -y}, {x: x, y: -y}, {x: x, y: y}, {x: -x, y: y}] : [
			{x: -x + c, y: -y},
			{x: x - c, y: -y},
			{x: x, y: -y + c},
			{x: x, y: y - c},
			{x: x - c, y: y},
			{x: -x + c, y: y},
			{x: -x, y: y - c},
			{x: -x, y: -y + c}
		];
		var base = footWidth > 0 ? footHeight : 0;
		var result:Array<CastingPrism> = [
			{
				id: "body",
				profile: profile,
				height: height - base,
				pose: F.translation(0, 0, base)
			}
		];
		if (footWidth > 0) {
			var f = footWidth / 2;
			result.push({
				id: "foot",
				profile: [
					{x: -f, y: 0},
					{x: f, y: 0},
					{x: f, y: footShoulder},
					{x: x, y: footHeight},
					{x: -x, y: footHeight},
					{x: -f, y: footShoulder}
				],
				height: depth,
				pose: F.fromRotationMatrix(0, y, 0, [1, 0, 0, 0, 0, -1, 0, 1, 0])
			});
		}
		return result;
	}
}

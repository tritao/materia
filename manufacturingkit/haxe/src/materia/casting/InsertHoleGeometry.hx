package materia.casting;

import materia.casting.InsertHole;

/** Explicit access for the single-hole box template; patterns use the full hole array. */
class InsertHoleGeometry {
	public static function single(holes:Array<InsertHole>):Null<InsertHole> {
		if (holes.length > 1)
			throw "Single-hole template cannot represent this insert hole pattern";
		return holes.length == 0 ? null : holes[0];
	}

	public static function diameter(holes:Array<InsertHole>):Float {
		var h = single(holes);
		return h == null ? 0 : h.diameter;
	}

	public static function through(diameter:Float, depth:Float):Array<InsertHole> {
		if (diameter == 0)
			return [];
		return [
			{
				id: "fixing",
				frame: materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, depth, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
				diameter: diameter,
				depth: depth,
				extent: Through,
				thread: null,
				tapDrill: null,
				counterbore: null
			}
		];
	}
}

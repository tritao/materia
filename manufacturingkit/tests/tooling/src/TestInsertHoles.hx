import materia.casting.InsertHole;
class TestInsertHoles {
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
				threadDepth: null,
				clearanceDiameter: diameter,
				counterbore: null,
				limits: {
					policyId: "test-clearance-hole-v1", threaded: null,
					minBack: diameter / 4, minWall: diameter / 2, minClearance: diameter / 10, overrides: []
				}
			}
		];
	}

}

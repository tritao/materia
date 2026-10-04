package cadkit.sheetmetal;

/** Return hems edit an isolated sheet graph and derive their bend from the authored hem dimensions. */
class SheetHem {
	public static function add(source:Sheet, wall:Int, edge:WallEdge, type:HemType, length:Float,
		gap:Float = 0, side:SheetSide = A, ?radius:Float, wallId:Int = 0, bendId:Int = 0):Sheet {
		var bulb = type == Teardrop && radius == null ? source.material.defaultRadius() : radius;
		var dimensions = HemGeometry.derive(type, length, gap, source.thickness(), bulb);
		return SheetFlange.add(source, wall, edge, length, side == A ? dimensions.angle : -dimensions.angle,
			dimensions.radius, Leg, BendOutside, 0, 0, 0, 0, 0, wallId, bendId);
	}
}

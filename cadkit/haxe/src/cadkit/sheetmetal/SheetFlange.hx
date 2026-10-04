package cadkit.sheetmetal;

/** Exact edge flange construction; all dimensions edit a fresh graph, never the source solid. */
class SheetFlange {
	public static function add(source:Sheet, wall:Int, edge:WallEdge, length:Float, angle:Float, radius:Float,
		lengthSpec:LengthSpec = Leg, position:FlangePosition = BendOutside, offset:Float = 0):Sheet {
		var leg = BendDimensions.leg(length, lengthSpec, source.thickness(), radius, angle);
		if (angle == 0)
			throw new SheetDiagnosis("sheet.invalid-angle", "An edge flange requires a nonzero bend angle");
		var shift = switch (position) {
			case MaterialInside: -BendDimensions.outsideSetback(source.thickness(), radius, angle);
			case MaterialOutside: -radius * Math.tan(Math.abs(angle) / 2);
			case BendOutside: 0.0;
			case Offset: offset;
		};
		if (position == MaterialOutside && Math.abs(angle) > 179 * Math.PI / 180)
			throw new SheetDiagnosis("sheet.sharp-length-angle", "Sharp flange alignment is undefined above 179 degrees");
		var original = source.wall(wall).outline;
		var boundary = original.segment(edge.loop, edge.segment);
		if (!boundary.isLine())
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "An edge flange requires a straight wall edge", ["wall:" + wall]);
		var outline = WallOutlineEditor.offset(original, edge, shift);
		var result = source.copy();
		var parent = result.updateWall(wall, outline);
		var child = result.addWall(WallOutline.rectangle(leg, outline.segment(edge.loop, edge.segment).length()));
		var bend = result.addBend(wall, edge, child.id, new WallEdge(3), angle, radius);
		return result;
	}
}

package cadkit.sheetmetal;

import cadkit.sheetmetal.FlangeGeometry.FlangeBoundary;

/** Exact edge flange construction; all dimensions edit a fresh graph, never the source solid. */
class SheetFlange {
	public static function add(source:Sheet, wall:Int, edge:WallEdge, length:Float, angle:Float, radius:Float,
		lengthSpec:LengthSpec = Leg, position:FlangePosition = BendOutside, offset:Float = 0,
		gap1:Float = 0, gap2:Float = 0, extend1:Float = 0, extend2:Float = 0, wallId:Int = 0, bendId:Int = 0,
		reliefType:ReliefType = None, reliefWidth:Float = 0, reliefDepth:Float = 0):Sheet {
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
		ReliefGeometry.validate(outline.segment(edge.loop, edge.segment), gap1, gap2, reliefType, reliefWidth, reliefDepth);
		var relief = reliefType == None ? 0.0 : reliefWidth;
		var span = FlangeGeometry.span(outline, edge, gap1 + relief, gap2 + relief);
		var relieved = ReliefGeometry.cut(span.outline, outline.segment(edge.loop, edge.segment), gap1, gap2,
			reliefType, reliefWidth, reliefDepth, "bend:" + bendId + ":wall:" + wall + ":edge:" + original.identity(edge.loop, edge.segment));
		var identity = span.outline.identity(span.edge.loop, span.edge.segment);
		var attachment:Null<WallEdge> = null; var names = relieved.identities();
		for (i in 0...names.length) for (j in 0...names[i].length) if (names[i][j] == identity) attachment = new WallEdge(j, i);
		if (attachment == null) throw new SheetDiagnosis("sheet.invalid-relief", "Relief removes the bend attachment", [identity]);
		span = new FlangeBoundary(relieved, attachment);
		var flange = FlangeGeometry.wall(leg, span.outline.segment(span.edge.loop, span.edge.segment).length(), extend1 + relief, extend2 + relief);
		var result = source.copy();
		var parent = result.updateWall(wall, span.outline);
		var child = wallId == 0 ? result.addWall(flange.outline) : result.insertWall(wallId, flange.outline);
		var bend = bendId == 0 ? result.addBend(wall, span.edge, child.id, flange.edge, angle, radius)
			: result.insertBend(bendId, wall, span.edge, child.id, flange.edge, angle, radius);
		return result;
	}
}

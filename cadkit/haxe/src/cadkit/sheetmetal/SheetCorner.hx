package cadkit.sheetmetal;

/** Close two selected flange ends and cut their relief as one verified material transaction. */
class SheetCorner {
	public static function apply(source:Sheet, first:SheetCornerEdge, second:SheetCornerEdge,
		join:CornerJoin, gap:Float, overlap:Float, relief:CornerRelief, reliefSize:Float, owner:String):Sheet {
		if (first.wall == second.wall)
			throw new SheetDiagnosis("sheet.corner-adjacency", "Corner closure requires different flange walls", ["wall:" + first.wall]);
		var firstBend = flangeBend(source, first.wall); var secondBend = flangeBend(source, second.wall);
		var cut = CornerReliefGeometry.cut(source, firstBend.id, secondBend.id, relief, reliefSize, owner);
		var a = source.wall(first.wall).outline; var b = source.wall(second.wall).outline;
		var firstEdge = first.resolve(source); var secondEdge = second.resolve(source);
		var folded = source.evaluate(source.folded());
		var offsets = CornerGeometry.of(a, firstEdge, folded.wall(first.wall).plane,
			b, secondEdge, folded.wall(second.wall).plane, source.thickness(), join, gap, overlap);
		return SheetCutGeometry.applyEdits(source, [
			new SheetContourEdit(first.wall, WallOutlineEditor.offset(a, firstEdge, offsets.first)),
			new SheetContourEdit(second.wall, WallOutlineEditor.offset(b, secondEdge, offsets.second))], [cut]);
	}
	static function flangeBend(source:Sheet, wall:Int):Bend {
		var selected:Null<Bend> = null;
		for (bend in source.bends()) if (bend.wallB == wall && !source.isRipped(bend.id)) {
			if (selected != null) throw new SheetDiagnosis("sheet.corner-adjacency", "Corner flange has more than one incoming bend", ["wall:" + wall]);
			selected = bend;
		}
		if (selected == null) throw new SheetDiagnosis("sheet.corner-adjacency", "Corner selection is not an active flange wall", ["wall:" + wall]);
		return selected;
	}
}

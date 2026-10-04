package cadkit.sheetmetal;

/** Exact wall-region subtraction; bend-zone trimming remains an explicit separate operation. */
class SheetWallCut {
	public static function apply(source:Sheet, wall:Int, tool:WallOutline, owner:String):Sheet {
		var outline = source.wall(wall).outline;
		for (bend in source.bends()) {
			var attachment:Null<WallEdge> = bend.wallA == wall ? bend.edgeA : bend.wallB == wall ? bend.edgeB : null;
			if (attachment == null) continue;
			var edge = outline.segment(attachment.loop, attachment.segment); var length = source.bendAllowance(bend);
			if (length <= 0) continue;
			var dx = (edge.end.y - edge.start.y) / edge.length() * length;
			var dy = (edge.start.x - edge.end.x) / edge.length() * length;
			var strip = WallOutline.polygon([edge.start, new Point2(edge.start.x + dx, edge.start.y + dy),
				new Point2(edge.end.x + dx, edge.end.y + dy), edge.end]);
			var crosses = false;
			try { var remaining = OutlineDifference.subtract(strip, tool, owner); crosses = remaining.area() < strip.area() - 1e-9; }
			catch (error:Dynamic) {
				var diagnosis = Std.downcast(error, SheetDiagnosis);
				if (diagnosis != null && (diagnosis.code == "sheet.cut-empty" || diagnosis.code == "sheet.cut-disconnected")) crosses = true;
				else throw error;
			}
			if (crosses) throw new SheetDiagnosis("sheet.cut-bend-zone", "Cut reaches a bend zone that requires exact developed trimming", ["bend:" + bend.id]);
		}
		var result = source.copy(); var changed = result.updateWall(wall, OutlineDifference.subtract(outline, tool, owner)); return result;
	}
}

package cadkit.sheetmetal;

/** One relief in the parent blank trims every adjoining developed wall and bend material patch. */
class CornerReliefGeometry {
	public static function apply(source:Sheet, firstBend:Int, secondBend:Int, type:CornerRelief,
		size:Float, owner:String):Sheet {
		return SheetCutGeometry.applyEdits(source, [], [cut(source, firstBend, secondBend, type, size, owner)]);
	}
	/** Keep the tool authored in the parent blank while a corner stages both flange contours. */
	public static function cut(source:Sheet, firstBend:Int, secondBend:Int, type:CornerRelief,
		size:Float, owner:String):SheetDevelopedCut {
		var first = source.bend(firstBend); var second = source.bend(secondBend);
		if (first.id == second.id || first.wallA != second.wallA || first.wallB == second.wallB
			|| source.isRipped(first.id) || source.isRipped(second.id))
			throw new SheetDiagnosis("sheet.corner-adjacency", "Corner relief requires two active flange bends on one parent wall", ["bend:" + firstBend, "bend:" + secondBend]);
		var a = first.edgeA.span; var b = second.edgeA.span;
		var ax = (a.end.x - a.start.x) / a.length(); var ay = (a.end.y - a.start.y) / a.length();
		var bx = (b.end.x - b.start.x) / b.length(); var by = (b.end.y - b.start.y) / b.length();
		var determinant = ax * by - ay * bx;
		if (Math.abs(determinant) < 1e-8)
			throw new SheetDiagnosis("sheet.corner-adjacency", "Parallel bend seams do not define a corner", ["bend:" + firstBend, "bend:" + secondBend]);
		var along = ((b.start.x - a.start.x) * by - (b.start.y - a.start.y) * bx) / determinant;
		var center = new Point2(a.start.x + along * ax, a.start.y + along * ay);
		return new SheetDevelopedCut(first.wallA, tool(center, ax, ay, type, size), owner, BoundaryOrigin.Relief(owner));
	}
	public static function tool(center:Point2, dx:Float, dy:Float, type:CornerRelief, size:Float):WallOutline {
		if (!Math.isFinite(size) || size <= 0 || !Math.isFinite(dx) || !Math.isFinite(dy)
			|| Math.abs(dx * dx + dy * dy - 1) > 1e-8)
			throw new SheetDiagnosis("sheet.invalid-corner-relief", "Corner relief requires a positive size and a unit square direction");
		var half = size / 2;
		return switch (type) {
			case Round: new WallOutline([Segment.arc(center, half, 0, Math.PI), Segment.arc(center, half, Math.PI, Math.PI)]);
			case Square: WallOutline.polygon([point(center, dx, dy, -half, -half), point(center, dx, dy, half, -half),
				point(center, dx, dy, half, half), point(center, dx, dy, -half, half)]);
		};
	}
	static function point(center:Point2, dx:Float, dy:Float, x:Float, y:Float):Point2 {
		return new Point2(center.x + dx * x - dy * y, center.y + dy * x + dx * y);
	}
}

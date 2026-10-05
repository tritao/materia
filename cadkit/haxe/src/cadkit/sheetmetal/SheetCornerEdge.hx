package cadkit.sheetmetal;

import cadkit.Shape;

/** Permanent wall and authored boundary identities select a corner without native enumeration. */
class SheetCornerEdge {
	public final wall:Int;
	public final identity:String;
	public function new(wall:Int, identity:String) {
		if (wall <= 0 || identity == null || identity.length == 0)
			throw new SheetDiagnosis("sheet.corner-selection", "Corner selection requires a wall and authored edge identity");
		this.wall = wall; this.identity = identity;
	}
	public function resolve(sheet:Sheet):WallEdge {
		var names = sheet.wall(wall).outline.identities();
		for (i in 0...names.length) for (j in 0...names[i].length)
			if (names[i][j] == identity) return new WallEdge(j, i);
		throw new SheetDiagnosis("sheet.corner-selection", "Corner edge no longer exists on its authored wall", ["wall:" + wall, identity]);
	}
	/** Resolve a picked side face once into graph-owned identities; native names remain opaque. */
	public static function fromFace(sheet:Sheet, face:Shape):SheetCornerEdge {
		var selected = SheetEdgeSelection.identify(sheet, face);
		return new SheetCornerEdge(selected.wall, sheet.wall(selected.wall).outline.identity(selected.edge.loop, selected.edge.segment));
	}
}

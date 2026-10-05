package cadkit.sheetmetal;

/** An exact cut in one wall's developed coordinates, with an explicit authored purpose. */
class SheetDevelopedCut {
	public final wall:Int;
	public final tool:WallOutline;
	public final owner:String;
	public final origin:BoundaryOrigin;
	public function new(wall:Int, tool:WallOutline, owner:String, ?origin:BoundaryOrigin) {
		if (owner == null || owner.length == 0)
			throw new SheetDiagnosis("sheet.cut-identity", "Developed cut requires an authored identity");
		this.wall = wall; this.tool = tool; this.owner = owner;
		this.origin = origin == null ? BoundaryOrigin.Cut(owner) : origin;
	}
}

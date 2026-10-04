package cadkit.sheetmetal;

/** Exact cut tool in developed bend coordinates, in millimetres, with an authored naming role. */
class BendCut {
	public final region:WallOutline;
	public final owner:String;
	public function new(region:WallOutline, owner:String) {
		if (owner == null || owner.length == 0) throw new SheetDiagnosis("sheet.cut-identity", "Bend cut requires an authored identity");
		this.region = region; this.owner = owner;
	}
}

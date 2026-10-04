package cadkit.sheetmetal;

/** A permanent connection between two nominal straight wall seams, with angle in radians. */
class Bend {
	public final id:Int;
	public final wallA:Int;
	public final edgeA:BendAttachment;
	public final wallB:Int;
	public final edgeB:BendAttachment;
	public final angle:Float;
	public final radius:Float;
	final cuts:Array<BendCut>;
	public function new(id:Int, wallA:Int, edgeA:BendAttachment, wallB:Int, edgeB:BendAttachment, angle:Float, radius:Float, ?cuts:Array<BendCut>) {
		this.id = id;
		this.wallA = wallA;
		this.edgeA = edgeA;
		this.wallB = wallB;
		this.edgeB = edgeB;
		this.angle = angle;
		this.radius = radius;
		this.cuts = cuts == null ? [] : cuts.copy();
	}
	public function cutTools():Array<BendCut> { return cuts.copy(); }
	public function hasCuts():Bool { return cuts.length > 0; }
	/** Material regions derive from current allowance and authored cuts on every evaluation. */
	public function materialRegions(allowance:Float):Array<WallOutline> {
		if (!Math.isFinite(allowance) || allowance <= 0) throw new SheetDiagnosis("sheet.zero-bend-allowance", "Bend material needs positive developed length", ["bend:" + id]);
		var width = edgeA.span.length();
		var outline = new WallOutline(WallOutline.rectangle(allowance, width).loops()[0], [],
			[["bend:" + id + ":end:0", "bend:" + id + ":join:b", "bend:" + id + ":end:1", "bend:" + id + ":join:a"]]);
		var regions = [outline];
		for (cut in cuts) {
			var next:Array<WallOutline> = [];
			for (region in regions) for (part in OutlineDifference.regions(region, cut.region, cut.owner)) next.push(part);
			regions = next;
		}
		return regions;
	}
}

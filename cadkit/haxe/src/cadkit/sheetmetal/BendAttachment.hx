package cadkit.sheetmetal;

/** Nominal straight seam in a wall frame; material cuts may interrupt it without moving the bend datum. */
class BendAttachment {
	public final identity:String;
	public final span:Segment;
	public function new(identity:String, span:Segment) {
		if (identity == null || identity.length == 0 || !span.isLine())
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "A bend attachment needs an authored identity and a straight span");
		this.identity = identity; this.span = span;
	}
	public static function fromOutline(outline:WallOutline, edge:WallEdge):BendAttachment {
		return new BendAttachment(outline.identity(edge.loop, edge.segment), outline.segment(edge.loop, edge.segment));
	}
}

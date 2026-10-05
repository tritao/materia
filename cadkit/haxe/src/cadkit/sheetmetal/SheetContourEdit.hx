package cadkit.sheetmetal;

/** One staged material contour; nominal bend datums remain owned by the source graph. */
class SheetContourEdit {
	public final wall:Int;
	public final outline:WallOutline;
	public function new(wall:Int, outline:WallOutline) { this.wall = wall; this.outline = outline; }
}

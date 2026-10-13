package cadkit.sheetmetal;

/** A permanent graph identity with an exact wall-local region. Edits replace data, preserving the id. */
class Wall {
	public final id:Int;
	public final outline:WallOutline;
	public function new(id:Int, outline:WallOutline) {
		this.id = id;
		this.outline = outline;
	}
}

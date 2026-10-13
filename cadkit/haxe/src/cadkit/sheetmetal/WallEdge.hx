package cadkit.sheetmetal;

/** An authored boundary segment. Indices belong to the wall outline, never native face enumeration. */
class WallEdge {
	public final loop:Int;
	public final segment:Int;
	public function new(segment:Int, loop:Int = 0) {
		this.loop = loop;
		this.segment = segment;
	}
	public function key():String {
		return loop + ":" + segment;
	}
}

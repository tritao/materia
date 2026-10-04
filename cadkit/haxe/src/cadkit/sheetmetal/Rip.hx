package cadkit.sheetmetal;

/** A zero-gap cut at the child-side seam removes connectivity; the bend strip remains on wall A. */
class Rip {
	public final bend:Int;
	public function new(bend:Int) {
		this.bend = bend;
	}
}

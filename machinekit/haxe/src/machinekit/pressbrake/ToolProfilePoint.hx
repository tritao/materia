package machinekit.pressbrake;

/** A tooling section point in millimetres: across the throat and above the tip. */
class ToolProfilePoint {
	public final y:Float;
	public final z:Float;

	public function new(y:Float, z:Float) {
		if (!Math.isFinite(y) || !Math.isFinite(z)) throw "Tool profile point needs finite coordinates";
		this.y = y;
		this.z = z;
	}
}

package machinekit.pressbrake.planning;

/** A finite travel interval in mm. A fixed optional axis has equal endpoints. */
class PressTravel {
	public final lower:Float;
	public final upper:Float;
	public function new(lower:Float, upper:Float) {
		if (!Math.isFinite(lower) || !Math.isFinite(upper) || upper < lower) throw "Invalid press travel interval";
		this.lower = lower; this.upper = upper;
	}
}

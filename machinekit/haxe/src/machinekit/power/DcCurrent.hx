package machinekit.power;

/** DC supply amperes; distinct from a motor winding's phase RMS current. */
abstract DcCurrent(Float) {
	public function new(amps:Float) {
		if (!(amps > 0) || !Math.isFinite(amps)) throw "DC current must be finite and positive";
		this = amps;
	}
	public function amps():Float return this;
}

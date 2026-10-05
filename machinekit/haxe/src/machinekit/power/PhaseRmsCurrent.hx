package machinekit.power;

/** Motor winding phase RMS amperes; distinct from DC supply current. */
abstract PhaseRmsCurrent(Float) {
	public function new(amps:Float) {
		if (!(amps > 0) || !Math.isFinite(amps)) throw "Phase RMS current must be finite and positive";
		this = amps;
	}
	public function amps():Float return this;
}

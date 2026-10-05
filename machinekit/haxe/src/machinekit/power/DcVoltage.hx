package machinekit.power;

/** DC volts. Raw values enter at recipe and native boundaries explicitly. */
abstract DcVoltage(Float) {
	public function new(volts:Float) {
		if (!(volts > 0) || !Math.isFinite(volts)) throw "DC voltage must be finite and positive";
		this = volts;
	}
	public function volts():Float return this;
}

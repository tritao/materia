package machinekit.power;

/** AC RMS volts. Deliberately has no implicit conversion to or from DC volts. */
abstract AcRmsVoltage(Float) {
	public function new(volts:Float) {
		if (!(volts > 0) || !Math.isFinite(volts)) throw "AC RMS voltage must be finite and positive";
		this = volts;
	}
	public function volts():Float return this;
}

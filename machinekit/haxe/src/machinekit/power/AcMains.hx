package machinekit.power;

/** The voltage convention is part of the mains system, not inferred from a number. */
enum AcMainsSystem {
	SinglePhase;
	ThreePhase;
}

/** AC supply boundary. Three-phase voltage is line-to-line RMS; single-phase
 * voltage is line-to-neutral RMS. Frequency is in hertz. No neutral is assumed
 * for the three-phase rectifier's three phases and protective earth.
 */
class AcMains {
	public final system:AcMainsSystem;
	public final rmsVoltage:AcRmsVoltage;
	public final frequencyHz:Float;
	public function new(system:AcMainsSystem, rmsVoltage:AcRmsVoltage, frequencyHz:Float) {
		if (!(frequencyHz > 0) || !Math.isFinite(frequencyHz)) throw "Mains frequency must be finite and positive";
		this.system = system;
		this.rmsVoltage = rmsVoltage;
		this.frequencyHz = frequencyHz;
	}
	/** Nominal capacitor-bus peak; conversion explicitly changes the voltage kind. */
	public function capacitorBus():DcVoltage return new DcVoltage(Math.sqrt(2.0) * rmsVoltage.volts());
}

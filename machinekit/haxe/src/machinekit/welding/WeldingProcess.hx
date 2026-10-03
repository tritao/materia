package machinekit.welding;

/** An arc process a welding power source can run. */
enum WeldingProcess {
	/** Gas metal arc with an inert shielding gas. */
	Mig;
	/** Gas metal arc with an active shielding gas, such as argon with CO2. */
	Mag;
	FluxCored;
}

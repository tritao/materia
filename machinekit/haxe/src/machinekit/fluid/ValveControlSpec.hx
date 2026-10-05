package machinekit.fluid;

/** Directional spool behaviour exposed by a fluid valve component. */
enum ValveControlSpec {
	PressureEffort(doubleSolenoid:Bool, normallyToA:Bool);
	ClosedCentre;
}

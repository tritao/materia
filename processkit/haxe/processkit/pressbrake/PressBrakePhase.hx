package processkit.pressbrake;

/** Measured handshake phases, independent of controller or simulation hosting. */
enum PressBrakePhase {
	Idle;
	ClosingToClamp;
	Clamped;
	Withdrawing;
	Working;
	Bottom;
	Decompressing;
	Opening;
	Fault;
}

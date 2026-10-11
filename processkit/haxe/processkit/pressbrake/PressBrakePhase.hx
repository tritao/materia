package processkit.pressbrake;

/** Measured handshake phases, independent of controller or simulation hosting. */
enum PressBrakePhase {
	Idle;
	Positioning;
	ClosingToClamp;
	Clamped;
	Withdrawing;
	Working;
	Bottom;
	Decompressing;
	Opening;
	Fault;
}

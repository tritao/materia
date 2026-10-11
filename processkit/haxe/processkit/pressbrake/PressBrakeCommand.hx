package processkit.pressbrake;

/** Requests to the press hydraulic controller; hardware retains final valve authority. */
enum PressBrakeCommand {
	Hold;
	CloseToClamp;
	CloseToDepth;
	Decompress;
	Open;
}

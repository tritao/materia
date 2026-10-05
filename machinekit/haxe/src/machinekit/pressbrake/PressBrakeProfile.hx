package machinekit.pressbrake;

/** The ram positioning mechanism determines what its controller can command. */
enum PressBrakeProfile {
	/** One mechanical depth stop sets both sides, joined by a torsion bar. */
	StopControlled;
	/** Each cylinder follows its own measured ram position. */
	ClosedLoopY1Y2;
}

package machinekit.component;

/** Geometry fidelity requested from a component generator. */
enum ComponentDetail {
	/** Standard outer envelope only: cheap, correct for clearance and packaging. */
	Envelope;
	/** Recognizable exterior features with simplified internals. */
	Preview;
}

package machinekit.component;

/** Geometry fidelity requested from a component generator. */
enum ComponentDetail {
	/** Outer envelope for clearance and packaging; moving parts cover their full travel. */
	Envelope;
	/** Recognizable exterior features with simplified internals. */
	Preview;
}

package machinekit.fluid;

/** Cylinder flow law; setup owns the phase profile of a hydraulic binding. */
enum CylinderControlSpec {
	PressureEffort(ratedSpeedMmPerSecond:Float);
	FlowLimited;
}

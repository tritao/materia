package humankit;

/** A limb inverse kinematics can pose: each arm ends at its wrist, each leg at its ankle. */
enum abstract HumanLimb(Int) from Int to Int {
	var ArmL = 0;
	var ArmR = 1;
	var LegL = 2;
	var LegR = 3;
}

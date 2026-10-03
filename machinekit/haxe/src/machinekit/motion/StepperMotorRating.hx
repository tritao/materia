package machinekit.motion;

/**
 * Electrical and mechanical ratings of a stepper motor model, by designation, in SI units:
 * holding torque in N m, rated phase current in A, phase inductance in H, rotor inertia in
 * kg m², full-step angle in degrees.
 */
typedef StepperMotorRating = {
	var designation:String;
	var holdingTorque:Float;
	var ratedCurrent:Float;
	var phaseInductance:Float;
	var rotorInertia:Float;
	var stepAngle:Float;
}

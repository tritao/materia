package machinekit.motion;

/** Assumed three-phase winding data. Torque constant uses phase RMS current;
 * back-EMF constant is line-to-line RMS volts per rad/s. They are stated separately.
 * Phase resistance is ohms, and mechanical efficiency excludes copper loss.
 */
typedef ServoElectricalRating = {
	var torqueConstant:Float;
	var backEmfConstant:Float;
	var phaseResistance:Float;
	var mechanicalEfficiency:Float;
}

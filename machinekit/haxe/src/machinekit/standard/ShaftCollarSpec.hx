package machinekit.standard;

/** Set-screw shaft collar dimensions in millimetres. `setScrew` is the clamping screw size;
 * its hole is not modelled, matching the project's semantic (not geometric) thread convention.
 */
typedef ShaftCollarSpec = {
	var boreDiameter:Float;
	var outerDiameter:Float;
	var width:Float;
	var setScrew:String;
}

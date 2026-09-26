package machinekit.motion;

/** Nominal NEMA frame mounting interface in millimetres. Motor bodies and shafts vary by model. */
typedef NemaFrameInterface = {
	var frame:Int;
	var face:Float;
	var boltSpacing:Float;
	var pilotDiameter:Float;
}

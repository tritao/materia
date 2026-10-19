package machinekit.motion;

/** Leadshine ELM1/ELM2 drawing dimensions, mm; catalog currents in Arms, speeds in rpm. */
typedef LeadshineServoSpec = {
	var designation:String;
	var series:String;
	var brake:Bool;
	var watts:Int;
	var voltage:Float;
	var ratedRpm:Float;
	var maxRpm:Float;
	var face:Float;
	var bodyLength:Float;
	var shaftDiameter:Float;
	var shaftLength:Float;
	var boltCircle:Float;
	var holeDiameter:Float;
	var flangeThickness:Float;
	var pilotDiameter:Float;
	var pilotHeight:Float;
	/** Maximum radial connector extent LH, measured from the shaft axis. */
	var connectorHeight:Float;
	var keyLength:Float;
	var keyWidth:Float;
	var keyHeight:Float;
	/** KH: distance across the shaft from the keyway floor to the opposite surface. */
	var keyFloor:Float;
	var ratedTorque:Float;
	var peakTorque:Float;
	var ratedCurrent:Float;
	var peakCurrent:Float;
	var rotorInertia:Float;
	var mass:Float;
}

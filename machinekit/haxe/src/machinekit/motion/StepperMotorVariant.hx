package machinekit.motion;

/** Dimensions of a specific stepper motor model, separate from its frame mounting pattern. */
typedef StepperMotorVariant = {
	var designation:String;
	var frame:Int;
	var bodyFace:Float;
	var bodyLength:Float;
	var shaftDiameter:Float;
	var shaftLength:Float;
	var pilotHeight:Float;
	var mountScrew:String;
	var tappedMount:Bool;
	/** Axial depth of the mounting flange (or blind tapped hole). */
	var mountHoleDepth:Float;
	/** Manufacturer clearance bore, when specified independently of screw fit. */
	var ?mountHoleDiameter:Float;
	/** Named, dimension-checked construction profile; absent uses the generic envelope. */
	var ?housingProfile:String;
}

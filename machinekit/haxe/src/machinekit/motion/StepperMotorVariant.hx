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
	var mountHoleDepth:Float;
}

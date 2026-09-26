package machinekit.motion;

/** Catalog dimensions for a base-mounted UCP-style pillow block unit, in mm. */
typedef PillowBlockSpec = {
	var designation:String;
	var family:String;
	var bearingDesignation:String;
	var boreDiameter:Float;
	var baseWidth:Float;
	var length:Float;
	var shaftHeight:Float;
	var baseHeight:Float;
	var overallHeight:Float;
	var boltSpacing:Float;
	var mountHoleDiameter:Float;
	var mountScrew:String;
}

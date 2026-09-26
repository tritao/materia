package machinekit.motion;

/** Catalog dimensions for a profile rail and its matching carriage block. Values are in mm. */
typedef LinearRailProfileSpec = {
	var designation:String;
	var family:String;
	var railWidth:Float;
	var railHeight:Float;
	var blockWidth:Float;
	var blockHeight:Float;
	var blockLength:Float;
	/** Distance between the two block mounting holes along the rail axis. */
	var blockHoleSpacing:Float;
	var blockMountScrew:String;
	var railHolePitch:Float;
	var railEndMargin:Float;
	var railMountScrew:String;
}

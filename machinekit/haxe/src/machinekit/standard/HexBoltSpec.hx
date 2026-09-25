package machinekit.standard;

/** ISO 4017 coarse-thread hex bolt dimensions in millimetres. Clearance holes follow ISO 273. */
typedef HexBoltSpec = {
	var size:String;
	var diameter:Float;
	var pitch:Float;
	var acrossFlats:Float;
	var headHeight:Float;
	/** Reference thread length b; shorter bolts are fully threaded. */
	var threadLength:Float;
	var tapDrill:Float;
	var clearanceFine:Float;
	var clearanceMedium:Float;
	var clearanceCoarse:Float;
}

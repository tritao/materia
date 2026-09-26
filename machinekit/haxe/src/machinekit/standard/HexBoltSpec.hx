package machinekit.standard;

/** ISO 4017 (fully threaded) coarse-thread hex bolt dimensions in millimetres. Clearance holes
 * follow ISO 273.
 */
typedef HexBoltSpec = {
	var size:String;
	var diameter:Float;
	var pitch:Float;
	var acrossFlats:Float;
	var headHeight:Float;
	var tapDrill:Float;
	var clearanceFine:Float;
	var clearanceMedium:Float;
	var clearanceCoarse:Float;
}

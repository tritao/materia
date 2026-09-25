package machinekit.standard;

/** Metric coarse-thread screw dimensions in millimetres.
 * Head dimensions follow ISO 4762, clearance holes ISO 273, counterbores DIN 974-1.
 */
typedef MetricScrewSpec = {
	var size:String;
	var diameter:Float;
	var pitch:Float;
	var headDiameter:Float;
	var headHeight:Float;
	var socketSize:Float;
	var socketDepth:Float;
	/** Reference thread length b; shorter screws are fully threaded. */
	var threadLength:Float;
	var tapDrill:Float;
	var clearanceFine:Float;
	var clearanceMedium:Float;
	var clearanceCoarse:Float;
	var counterboreDiameter:Float;
	var counterboreDepth:Float;
}

package materia.tooling.casting;

/** Supplier EPS data, separate from a display material. Null means unknown.
 * Declared compression strength includes the strain at which it was measured;
 * it is not automatically an allowable stress for dimensional control.
 */
typedef CoreStockSpec = {
	var materialSpec:String;
	var width:Float;
	var depth:Float;
	var height:Float;
	var densityKgM3:Null<Float>;
	var compressionStrengthPa:Null<Float>;
	var compressionStrainFraction:Null<Float>;
	var maximumTemperatureC:Null<Float>;
	var evidence:Array<String>;
}

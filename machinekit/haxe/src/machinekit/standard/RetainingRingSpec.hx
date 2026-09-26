package machinekit.standard;

/** DIN 471 external (shaft) retaining ring dimensions in millimetres.
 * `grooveDiameter` is the mating `SteppedShaft` groove diameter.
 */
typedef RetainingRingSpec = {
	var shaftDiameter:Float;
	var grooveDiameter:Float;
	var outerDiameter:Float;
	/** Nominal axial groove width m, distinct from ring thickness s. */
	var grooveWidth:Float;
	var thickness:Float;
}

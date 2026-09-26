package machinekit.standard;

/** DIN 6885-1 form B parallel key cross-section, in millimetres.
 * `maxShaft` is the upper bound (inclusive) of the shaft diameter range this size fits.
 */
typedef ParallelKeySpec = {
	var maxShaft:Float;
	var width:Float;
	var height:Float;
	/** Depth of the seat cut into the shaft (t1). */
	var shaftDepth:Float;
	/** Depth of the seat cut into the hub or mating part (t2). */
	var hubDepth:Float;
}

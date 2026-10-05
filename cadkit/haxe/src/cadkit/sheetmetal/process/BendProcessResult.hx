package cadkit.sheetmetal.process;

/** Predicted dimensions are mm, angles radians and force kN. Radii refer to the inner surface. */
typedef BendProcessResult = {
	final innerRadius:Float;
	final loadedRadius:Float;
	final bendAllowance:Float;
	final targetAngle:Float;
	final loadedAngle:Float;
	final overbend:Float;
	final depth:Float;
	final analyticDepth:Float;
	final depthOffset:Float;
	/** A radius/depth measurement applied; force and flange minimum remain assumed. */
	final calibrated:Bool;
	final force:Float;
	final minimumFlange:Float;
	final radiusBend:Bool;
	final recommendedOpening:Bool;
}

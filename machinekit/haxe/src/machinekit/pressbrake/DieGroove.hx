package machinekit.pressbrake;

/** One V groove. Opening is the distance between shoulder centres, matching AirBending's datum.
 * The die top is z=0; shoulder circle centres are at z=-shoulderRadius.
 */
class DieGroove {
	public final vWidth:Float;
	public final includedAngle:Float;
	public final shoulderRadius:Float;

	public function new(vWidth:Float, includedAngle:Float, shoulderRadius:Float) {
		if (!Math.isFinite(vWidth) || !(vWidth > 0)) throw "Die groove needs a positive opening";
		if (!Math.isFinite(includedAngle) || !(includedAngle > 0 && includedAngle < Math.PI))
			throw "Die groove needs an included angle between zero and pi";
		if (!Math.isFinite(shoulderRadius) || shoulderRadius < 0)
			throw "Die groove needs a finite nonnegative shoulder radius";
		this.vWidth = vWidth;
		this.includedAngle = includedAngle;
		this.shoulderRadius = shoulderRadius;
		if (!(tangentY() > 0)) throw "Die groove shoulders consume its opening";
	}

	/** Positive-Y inner contact tangent, measured from the groove centre. */
	public function tangentY():Float return vWidth / 2 - shoulderRadius * Math.cos(includedAngle / 2);

	public function tangentZ():Float return -shoulderRadius * (1 - Math.sin(includedAngle / 2));

	/** Depth below the die top where the two straight flanks meet. */
	public function depth():Float return -tangentZ() + tangentY() / Math.tan(includedAngle / 2);
}

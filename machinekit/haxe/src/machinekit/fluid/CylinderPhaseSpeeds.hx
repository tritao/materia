package machinekit.fluid;

/** Flow-limited cylinder speeds in mm/s. The change point is extension from home in mm. */
class CylinderPhaseSpeeds {
	public final stroke:Float;
	public final approach:Float;
	public final working:Float;
	public final returning:Float;
	public final changePoint:Float;

	public function new(stroke:Float, approach:Float, working:Float, returning:Float, changePoint:Float) {
		if (!(stroke > 0 && approach > 0 && working > 0 && returning > 0)
			|| !Math.isFinite(stroke + approach + working + returning + changePoint)
			|| changePoint < 0 || changePoint > stroke)
			throw "Cylinder phase speeds need finite positive stroke and rates, with a change point within travel";
		this.stroke = stroke; this.approach = approach; this.working = working;
		this.returning = returning; this.changePoint = changePoint;
	}

	/** Select the working speed at the change point, including finite end-stop overtravel. */
	public function extensionSpeed(extension:Float):Float {
		if (!Math.isFinite(extension)) throw "Cylinder extension must be finite";
		return extension < changePoint ? approach : working;
	}
	public function maximumSpeed():Float return Math.max(approach, Math.max(working, returning));

	/** Ideal no-load extension time, split at the physical phase-change position. */
	public function extensionTime(from:Float, to:Float):Float {
		checkPosition(from); checkPosition(to);
		if (to < from) throw "Cylinder extension interval runs backwards";
		var fast = Math.max(0, Math.min(to, changePoint) - Math.min(from, changePoint));
		var slow = to - from - fast;
		return fast / approach + slow / working;
	}
	public function returnTime(from:Float, to:Float):Float {
		checkPosition(from); checkPosition(to);
		if (to > from) throw "Cylinder return interval runs forwards";
		return (from - to) / returning;
	}
	function checkPosition(position:Float):Void {
		if (!Math.isFinite(position) || position < 0 || position > stroke)
			throw "Cylinder timing position is outside its nominal travel";
	}
}

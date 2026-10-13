package machinekit.pressbrake.planning;

/** Contact datums of one installed station in the open press frame, in mm. */
class PressStationGeometry {
	public final id:String;
	public final dieTopHeight:Float;
	public final punchLineY:Float;
	public final openTipHeight:Float;
	public function new(id:String, dieTopHeight:Float, punchLineY:Float, openTipHeight:Float) {
		if (id == null || id.length == 0 || !Math.isFinite(dieTopHeight) || !Math.isFinite(punchLineY)
			|| !Math.isFinite(openTipHeight) || openTipHeight <= dieTopHeight) throw "Invalid open tooling station datums";
		this.id = id; this.dieTopHeight = dieTopHeight; this.punchLineY = punchLineY; this.openTipHeight = openTipHeight;
	}
}

package machinekit.pressbrake;

/** Rectangular end relief, retaining the contact nose up to `remainingHeight` above its tip. */
class ToolHorn {
	public final end:ToolHornEnd;
	public final reliefLength:Float;
	public final remainingHeight:Float;

	public function new(end:ToolHornEnd, reliefLength:Float, remainingHeight:Float) {
		if (end == null || !Math.isFinite(reliefLength) || !(reliefLength > 0)
			|| !Math.isFinite(remainingHeight) || !(remainingHeight > 0))
			throw "Tool horn needs an end and positive relief dimensions";
		this.end = end;
		this.reliefLength = reliefLength;
		this.remainingHeight = remainingHeight;
	}
}

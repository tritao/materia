package humankit;

/** Reaches a button, holds contact briefly, and releases the arm. */
class Press extends HumanActionBase {
	public var point(default, null):Array<Float>;
	final pointProvider:Null<Void->Array<Float>>;
	public final limb:HumanLimb;
	public final ramp:Float;
	public final hold:Float;
	var stage:Int = 0;
	var elapsed:Float = 0.0;

	public function new(point:Array<Float>, limb:HumanLimb = ArmR, ramp:Float = 0.2,
			hold:Float = 0.15, ?pointProvider:Void->Array<Float>) {
		super();
		this.point = point.copy();
		this.limb = limb;
		this.ramp = ramp;
		this.hold = hold;
		this.pointProvider = pointProvider;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (pointProvider != null) point = pointProvider().copy();
		if (point.length < 3 || ramp < 0.0 || hold < 0.0) {
			fail("Press needs a point and non-negative timing");
			return;
		}
		worker.setReachWorld(limb, point, 0.0);
	}

	override public function advance(seconds:Float):Void {
		elapsed += seconds;
		if (stage == 0) {
			var weight = ramp == 0.0 ? 1.0 : Math.min(1.0, elapsed / ramp);
			worker.setReachWorld(limb, point, weight);
			if (weight >= 1.0) { stage = 1; elapsed = 0.0; }
		} else if (stage == 1) {
			worker.setReachWorld(limb, point, 1.0);
			if (elapsed >= hold) { stage = 2; elapsed = 0.0; }
		} else {
			var weight = ramp == 0.0 ? 0.0 : Math.max(0.0, 1.0 - elapsed / ramp);
			if (weight <= 0.0) {
				worker.clearReach(limb);
				done = true;
			} else worker.setReachWorld(limb, point, weight);
		}
	}
}

package humankit;

/** Ramps an active IK limb back onto its animation. */
class ReleaseLimb extends HumanActionBase {
	public final limb:HumanLimb;
	public final ramp:Float;
	var target:Null<Array<Float>> = null;
	var pole:Null<Array<Float>> = null;
	var fromWeight:Float = 0.0;
	var elapsed:Float = 0.0;

	public function new(limb:HumanLimb, ramp:Float = 0.4) {
		super();
		this.limb = limb;
		this.ramp = ramp;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (ramp < 0.0) { fail("Release ramp cannot be negative"); return; }
		target = worker.reachTargetWorld(limb);
		pole = worker.reachPole(limb);
		fromWeight = worker.reachWeight(limb);
		if (target == null || fromWeight <= 0.0 || ramp == 0.0) {
			worker.clearReach(limb);
			done = true;
		}
	}

	override public function advance(seconds:Float):Void {
		if (done) return;
		elapsed += seconds;
		var weight = fromWeight * Math.max(0.0, 1.0 - elapsed / ramp);
		if (weight <= 0.0) {
			worker.clearReach(limb);
			done = true;
		} else {
			var point = target;
			if (point != null) worker.setReachWorld(limb, point, weight, pole);
		}
	}
}

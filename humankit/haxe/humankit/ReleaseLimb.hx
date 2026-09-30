package humankit;

/** Ramps an active IK limb back onto its animation. */
class ReleaseLimb extends HumanActionBase {
	public final limb:HumanLimb;
	/** The shortest the release takes; it lasts longer when the hand has far to go, to keep its speed down. */
	public final ramp:Float;
	var duration:Float = 0.0;
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
			return;
		}
		// Easing peaks at half as much again as the average speed, so the hand's travel sets how long it must take.
		duration = Math.max(ramp, 1.5 * worker.travelToAnimation(limb) / worker.posture.releaseSpeed);
	}

	override public function advance(seconds:Float):Void {
		if (done) return;
		elapsed += seconds;
		var progress = Math.min(1.0, elapsed / duration);
		var weight = fromWeight * (1.0 - progress * progress * (3.0 - 2.0 * progress));
		if (progress >= 1.0) {
			worker.clearReach(limb);
			done = true;
		} else {
			var point = target;
			if (point != null) worker.setReachWorld(limb, point, weight, pole);
		}
	}
}

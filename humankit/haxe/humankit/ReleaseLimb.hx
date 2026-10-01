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
	/** Set while the body is still rising from a crouch or a kneel: the release starts when it has stopped. */
	var waiting:Bool = false;

	public function new(limb:HumanLimb, ramp:Float = 0.4) {
		super();
		this.limb = limb;
		this.ramp = ramp;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (ramp < 0.0) { fail("Release ramp cannot be negative"); return; }
		// A hand let go while the body is still standing up has the animated pose it returns to moving away from it, which
		// speeds the hand up; the release waits, the hand staying where it is against the body, until the body is up.
		if (!worker.downReached() || worker.downAmount() > 0.001 || !worker.leanReached() || !worker.walker.settled()) {
			waiting = true;
			return;
		}
		begin();
	}

	function begin():Void {
		waiting = false;
		target = worker.reachTargetWorld(limb);
		pole = worker.reachPole(limb);
		fromWeight = worker.reachWeight(limb);
		if (target == null || fromWeight <= 0.0 || ramp == 0.0) {
			worker.clearReach(limb);
			done = true;
			return;
		}
		duration = worker.blendSeconds(worker.travelToAnimation(limb), ramp);
	}

	override public function advance(seconds:Float):Void {
		if (done) return;
		if (waiting) {
			if (!worker.downReached() || worker.downAmount() > 0.001 || !worker.leanReached() || !worker.walker.settled()) return;
			begin();
			if (done) return;
		}
		elapsed += seconds;
		var progress = Math.min(1.0, elapsed / duration);
		var weight = fromWeight * (1.0 - smooth(progress));
		if (progress >= 1.0) {
			worker.clearReach(limb);
			done = true;
		} else {
			var point = target;
			if (point != null) worker.setReachWorld(limb, point, weight, pole);
		}
	}
}

package humankit;

/** Phase of a HumanReachTask's walk-then-reach sequence. */
enum abstract HumanReachPhase(Int) from Int to Int {
	var Approaching = 0;
	var RampingIn = 1;
	var Holding = 2;
	var RampingOut = 3;
	var Finished = 4;
}

/** Legacy walk, reach, wait, and release convenience task. */
class HumanReachTask {
	public final walker:HumanWalker;
	public final limb:HumanLimb;
	public final target:Array<Float>;
	public final rampSeconds:Float;
	public final holdSeconds:Float;
	public var phase(default, null):HumanReachPhase = Approaching;

	final job:HumanJob;
	final reachIndex:Int;
	final waitIndex:Int;
	final releaseIndex:Int;

	/**
	 * The legacy target remains in model space. The composed job's walk uses
	 * floor-world points; its reach uses Reach.inModelSpace.
	 */
	public function new(walker:HumanWalker, route:Array<Array<Float>>, metresPerSecond:Float, limb:HumanLimb,
			target:Array<Float>, rampSeconds:Float = 0.4, holdSeconds:Float = 0.6, ?pole:Array<Float>,
			?clip:String, fadeSeconds:Float = 0.2) {
		if (!(rampSeconds > 0.0) || holdSeconds < 0.0)
			throw "A reach task needs a positive ramp and a non-negative hold";
		this.walker = walker;
		this.limb = limb;
		this.target = target.copy();
		this.rampSeconds = rampSeconds;
		this.holdSeconds = holdSeconds;
		if (clip != null && walker.character.asset.clipIndex(clip) < 0)
			throw 'The character needs a "$clip" clip';
		job = new HumanJob(new HumanBody(walker.character, walker));
		job.add(WalkTo.along(route, metresPerSecond));
		if (clip != null) job.add(new PlayClip(clip, 0.0, fadeSeconds));
		reachIndex = clip == null ? 1 : 2;
		waitIndex = reachIndex + 1;
		releaseIndex = waitIndex + 1;
		job.add(Reach.inModelSpace(limb, target, rampSeconds, pole));
		job.add(new Wait(holdSeconds));
		job.add(new ReleaseLimb(limb, rampSeconds));
		job.advance(0.0);
	}

	public function isDone():Bool
		return phase == Finished;

	public function advance(seconds:Float):Void {
		job.advance(seconds);
		var index = job.currentIndex();
		phase = job.isDone() ? Finished : index < reachIndex ? Approaching
			: index < waitIndex ? RampingIn : index < releaseIndex ? Holding : RampingOut;
	}
}

package humankit;

/** Phase of a HumanReachTask's walk-then-reach sequence. */
enum abstract HumanReachPhase(Int) from Int to Int {
	var Approaching = 0;
	var RampingIn = 1;
	var Holding = 2;
	var RampingOut = 3;
	var Finished = 4;
}

/**
 * Walks a character to a spot, stops, reaches a limb for a target with
 * AnimKit's two-bone IK, its weight ramped in and out over a short time
 * (optionally with a clip such as "interact" playing while it holds), and
 * releases the limb cleanly. A small state machine layered on HumanWalker,
 * driven the same way: construct it, then call advance() every frame.
 */
class HumanReachTask {
	public final walker:HumanWalker;
	public final limb:HumanLimb;
	public final target:Array<Float>;
	/** Seconds to ramp the IK weight from 0 to 1, and from 1 back to 0. */
	public final rampSeconds:Float;
	/** Seconds held at full IK weight before releasing. */
	public final holdSeconds:Float;
	final pole:Null<Array<Float>>;
	final clipIndex:Int;
	final fadeSeconds:Float;

	public var phase(default, null):HumanReachPhase = Approaching;
	var elapsed:Float = 0.0;

	/**
	 * Starts walker along route at speed. Once it arrives and stops, this
	 * reaches limb for target (a model-space point, as HumanCharacter.reach),
	 * ramping the IK weight in, holding, then ramping it out and releasing.
	 * clip, if given, plays (crossfading over fadeSeconds) once the walk
	 * stops; the character needs it, as HumanWalker needs its walk and idle
	 * clips.
	 */
	public function new(walker:HumanWalker, route:Array<Array<Float>>, metresPerSecond:Float, limb:HumanLimb,
			target:Array<Float>, rampSeconds:Float = 0.4, holdSeconds:Float = 0.6, ?pole:Array<Float>,
			?clip:String, fadeSeconds:Float = 0.2) {
		if (!(rampSeconds > 0.0) || holdSeconds < 0.0)
			throw "A reach task needs a positive ramp and a non-negative hold";
		this.walker = walker;
		this.limb = limb;
		this.target = target;
		this.rampSeconds = rampSeconds;
		this.holdSeconds = holdSeconds;
		this.pole = pole;
		this.fadeSeconds = fadeSeconds;
		if (clip != null) {
			var index = walker.character.asset.clipIndex(clip);
			if (index < 0)
				throw 'The character needs a "$clip" clip';
			clipIndex = index;
		} else
			clipIndex = -1;
		walker.follow(route, metresPerSecond);
	}

	public function isDone():Bool
		return phase == Finished;

	/** Advances the walk, and once arrived, the reach's ramp, hold, and release, by seconds. */
	public function advance(seconds:Float):Void {
		if (phase == Approaching) {
			walker.advance(seconds);
			if (!walker.isWalking()) {
				phase = RampingIn;
				elapsed = 0.0;
				if (clipIndex >= 0)
					walker.character.player.play(clipIndex, fadeSeconds);
			}
			return;
		}
		if (phase == Finished) {
			walker.advance(seconds);
			return;
		}
		elapsed += seconds;
		var weight = 1.0;
		if (phase == RampingIn) {
			weight = Math.min(1.0, elapsed / rampSeconds);
			if (weight >= 1.0) {
				phase = Holding;
				elapsed = 0.0;
			}
		} else if (phase == Holding) {
			if (elapsed >= holdSeconds) {
				phase = RampingOut;
				elapsed = 0.0;
				weight = 1.0;
			}
		} else if (phase == RampingOut)
			weight = Math.max(0.0, 1.0 - elapsed / rampSeconds);
		walker.character.reach(limb, target, weight, pole);
		walker.advance(seconds);
		if (phase == RampingOut && weight <= 0.0) {
			walker.character.release(limb);
			phase = Finished;
		}
	}
}

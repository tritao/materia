package humankit;

/**
 * Animation and locomotion state shared by every action in a worker's job.
 *
 * The body is what actions talk to: where it stands, which limbs reach or carry, how the hands
 * grip, how far the upper body leans. Each limb keeps its own small state machine
 * (`LimbControl`); the body steps them, solves wrist goals against the animated pose, and turns
 * the tuning in `HumanPosture` into the pose. See BODY.md for how the layers fit together.
 */
class HumanBody {
	public final character:HumanCharacter;
	public final walker:HumanWalker;
	public final description:HumanDescription;
	public final posture:HumanPosture;
	/** True between a completed pick and its matching place. */
	public var grip(default, null):Bool = false;

	/** The four limbs, indexed by HumanLimb. */
	final limbs:Array<LimbControl>;
	final heldPoints:Array<Null<Void->Array<Float>>> = [null, null, null, null];
	var leanGoal:Float = 0.0;
	var leanNow:Float = 0.0;

	public function new(character:HumanCharacter, ?walker:HumanWalker, ?posture:HumanPosture) {
		this.character = character;
		this.walker = walker == null ? new HumanWalker(character) : walker;
		if (this.walker.character != character)
			throw "A human body needs its character's walker";
		this.posture = posture == null ? HumanPosture.standard() : posture;
		description = HumanDescription.measure(character.pose, character.height());
		limbs = [for (limb in [ArmL, ArmR, LegL, LegR]) new LimbControl(limb, this.posture.relaxedCurl)];
		for (hand in [ArmL, ArmR]) character.setHandCurl(hand, this.posture.relaxedCurl);
	}

	/** Open while reaching for something, closed while holding it, relaxed otherwise. */
	function curlFor(hand:HumanLimb):Float {
		var control = limbs[hand];
		if (control.mode == Carry || heldPoints[hand] != null) return posture.gripCurl;
		return control.mode == Reach && control.weight > 0.0 ? posture.openCurl : posture.relaxedCurl;
	}

	/** Where the torso is asked to lean; it eases there. Zero stands upright. */
	public function setLean(angle:Float):Void
		leanGoal = Math.max(0.0, Math.min(posture.maxLean, angle));

	/**
	 * The lean that carries a hand's shoulder `shift` metres further forward, and the shift it can
	 * deliver (less than asked when that would pass the posture's maximum lean). Measured by leaning
	 * the skeleton a test amount, so it holds for any rig; the pose is left as it was.
	 */
	public function leanFor(hand:HumanLimb, shift:Float):{angle:Float, shift:Float} {
		var none = {angle: 0.0, shift: 0.0};
		if (!(shift > 1e-4)) return none;
		var bone = hand == ArmL ? HumanBone.UpperArmL : HumanBone.UpperArmR;
		var saved = character.spineLean();
		var probe = 0.2;
		character.setSpineLean(0.0);
		character.advance(0.0);
		var upright = character.pose.bonePosition(bone);
		character.setSpineLean(probe);
		character.advance(0.0);
		var leaned = character.pose.bonePosition(bone);
		character.setSpineLean(saved);
		character.advance(0.0);
		if (upright == null || leaned == null || leaned[0] - upright[0] < 1e-4) return none;
		var perRadian = (leaned[0] - upright[0]) / probe;
		var angle = Math.min(posture.maxLean, shift / perRadian);
		return {angle: angle, shift: angle * perRadian};
	}

	function moveLean(seconds:Float):Void {
		var step = posture.leanRate * seconds;
		leanNow = Math.abs(leanGoal - leanNow) <= step ? leanGoal : leanNow + (leanGoal > leanNow ? step : -step);
		character.setSpineLean(leanNow);
	}

	function moveFingers(seconds:Float):Void {
		for (hand in [ArmL, ArmR]) {
			var control = limbs[hand];
			var step = posture.curlRate * seconds, wanted = curlFor(hand);
			control.curl = Math.abs(wanted - control.curl) <= step ? wanted :
				control.curl + (wanted > control.curl ? step : -step);
			character.setHandCurl(hand, control.curl);
		}
	}

	public function rootTransform():Array<Float>
		return walker.rootTransform();

	public function toModel(worldPoint:Array<Float>):Array<Float> {
		var inverse = Mat4.rigidInverse(rootTransform());
		return transformPoint(inverse, worldPoint);
	}

	public function toWorld(modelPoint:Array<Float>):Array<Float>
		return transformPoint(rootTransform(), modelPoint);

	static function transformPoint(matrix:Array<Float>, point:Array<Float>):Array<Float>
		return [
			matrix[0] * point[0] + matrix[4] * point[1] + matrix[8] * point[2] + matrix[12],
			matrix[1] * point[0] + matrix[5] * point[1] + matrix[9] * point[2] + matrix[13],
			matrix[2] * point[0] + matrix[6] * point[1] + matrix[10] * point[2] + matrix[14]
		];

	public function setReachWorld(limb:HumanLimb, target:Array<Float>, weight:Float,
			?pole:Array<Float>):Void
		limbs[limb].reach(target, false, weight, pole);

	/** Legacy model-space reach target used by HumanReachTask. */
	public function setReachModel(limb:HumanLimb, target:Array<Float>, weight:Float,
			?pole:Array<Float>):Void
		limbs[limb].reach(target, true, weight, pole);

	public function reachWeight(limb:HumanLimb):Float
		return limbs[limb].weight;

	/** Predicted world position of the object reference point held by a hand. */
	public function setHeldPoint(hand:HumanLimb, provider:Null<Void->Array<Float>>):Void {
		if (hand != ArmL && hand != ArmR) throw "A held point needs an arm";
		heldPoints[hand] = provider;
	}

	public function heldPoint(hand:HumanLimb):Null<Array<Float>> {
		var provider = heldPoints[hand];
		return provider == null ? null : provider();
	}

	/**
	 * World position of a hand's palm centre, midway between the wrist and the
	 * middle-finger knuckle (the wrist itself on a rig without fingers). Pick
	 * brings this point to its grasp point.
	 */
	public function gripPoint(hand:HumanLimb):Array<Float> {
		var wrist = character.pose.bonePosition(hand == ArmL ? HandL : HandR);
		if (wrist == null) throw "The character has no hand bone";
		var knuckle = character.pose.bonePosition(hand == ArmL ? MiddleL : MiddleR);
		return toWorld(knuckle == null ? wrist : [for (axis in 0...3) (wrist[axis] + knuckle[axis]) * 0.5]);
	}

	/** Why a wrist goal is beyond 95% of the arm's length from its shoulder, or null. */
	public function reachFailure(hand:HumanLimb, goal:Array<Float>):Null<String> {
		var shoulder = character.pose.bonePosition(hand == ArmL ? UpperArmL : UpperArmR);
		if (shoulder == null) return "The character has no shoulder bone";
		var fromShoulder = distance(toWorld(shoulder), goal);
		var limit = posture.reachLimit * (description.upperArm + description.forearm);
		return fromShoulder > limit ? 'out of reach (${fromShoulder} m from the shoulder, limit ${limit} m)' : null;
	}

	/**
	 * Finds the wrist goal that brings point() onto target with the arm at full
	 * IK weight, re-evaluating the pose at the current animation time between
	 * attempts, so nothing accumulates across frames. Each attempt steps the
	 * best goal so far by the remaining miss; a step that does not improve on it
	 * is halved instead of taken, so the returned error never exceeds the
	 * guess's and repeated solves cannot oscillate. guess is the first wrist
	 * goal; the goal never moves more than cap metres from it. Leaves the arm
	 * reaching for the returned goal. failure is set only when the guess itself
	 * is out of reach; steps beyond reach are treated as overshoots.
	 */
	public function solveReach(hand:HumanLimb, target:Array<Float>, point:Void->Array<Float>,
			guess:Array<Float>, cap:Float):{goal:Array<Float>, error:Float, failure:Null<String>} {
		var failure = reachFailure(hand, guess);
		if (failure != null) return {goal: guess.copy(), error: Math.POSITIVE_INFINITY, failure: failure};
		var best = guess.copy(), bestMiss = [0.0, 0.0, 0.0], bestError = Math.POSITIVE_INFINITY;
		var goal = guess.copy(), gain = 1.0;
		for (attempt in 0...8) {
			if (reachFailure(hand, goal) == null) {
				setReachWorld(hand, goal, 1.0);
				evaluate();
				var reached = point();
				var miss = [for (axis in 0...3) target[axis] - reached[axis]];
				var error = Math.sqrt(miss[0] * miss[0] + miss[1] * miss[1] + miss[2] * miss[2]);
				if (error < bestError) {
					best = goal;
					bestMiss = miss;
					bestError = error;
				} else
					gain *= 0.5;
			} else
				gain *= 0.5;
			if (bestError < 0.002) break;
			var next = [for (axis in 0...3) best[axis] + bestMiss[axis] * gain];
			var moved = distance(next, guess);
			if (moved > cap)
				next = [for (axis in 0...3) guess[axis] + (next[axis] - guess[axis]) * cap / moved];
			goal = next;
		}
		setReachWorld(hand, best, 1.0);
		evaluate();
		return {goal: best, error: bestError, failure: null};
	}

	static function distance(a:Array<Float>, b:Array<Float>):Float
		return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2) + Math.pow(a[2] - b[2], 2));

	public function reachPole(limb:HumanLimb):Null<Array<Float>>
		return limbs[limb].pole;

	public function reachTargetWorld(limb:HumanLimb):Null<Array<Float>> {
		var control = limbs[limb];
		if (control.mode != Reach) return null;
		return control.modelSpace ? toWorld(control.target) : control.target.copy();
	}

	public function clearReach(limb:HumanLimb):Void
		limbs[limb].release(character);

	public function setCarry(hands:Array<HumanLimb>):Void {
		for (control in limbs) if (control.mode == Carry && hands.indexOf(control.limb) < 0) clearReach(control.limb);
		for (hand in hands) {
			var control = limbs[hand];
			// A hand takes up the carry pose from where it is, not by jumping there.
			var wrist = character.pose.bonePosition(hand == ArmL ? HandL : HandR);
			control.carry(wrist);
			control.dropIk(character);
		}
	}

	public function isCarrying(limb:HumanLimb):Bool
		return limbs[limb].mode == Carry;

	/** Current carry target in model space, tied to the animated chest. */
	public function carryTargetModel(limb:HumanLimb):Array<Float> {
		var chest = character.pose.bonePosition(Chest);
		if (chest == null) chest = character.pose.bonePosition(Pelvis);
		if (chest == null) throw "The character has no chest or pelvis";
		var side = limb == ArmL ? 1.0 : -1.0;
		return [chest[0] + posture.carryOffset[0], chest[1] + side * posture.carryOffset[1],
			chest[2] + posture.carryOffset[2]];
	}

	public function setGrip(held:Bool):Void
		grip = held;

	public function cancel():Void {
		walker.stop();
		grip = false;
		for (hand in [ArmL, ArmR]) setHeldPoint(hand, null);
		for (control in limbs) control.release(character);
		for (hand in [ArmL, ArmR]) {
			limbs[hand].curl = posture.relaxedCurl;
			character.setHandCurl(hand, posture.relaxedCurl);
		}
		leanGoal = 0.0;
		leanNow = 0.0;
		character.setSpineLean(0.0);
	}

	/**
	 * Returns the body to rest at a floor pose as if newly constructed: no route,
	 * no reach, nothing carried or held, and the idle pose evaluated.
	 */
	public function reset(x:Float, y:Float, heading:Float):Void {
		cancel();
		walker.restart(x, y, heading);
		advance(0.0);
	}

	/** Advances gait once, then reapplies current world targets over that pose. */
	public function advance(seconds:Float):Void {
		for (control in limbs) control.advance(seconds, posture.carryEaseSeconds);
		moveFingers(seconds);
		moveLean(seconds);
		walker.advance(seconds);
		evaluate();
	}

	/** Re-evaluates reaches at the current animation time for in-frame IK solving. */
	public function evaluate():Void {
		// A lean swings an unused arm back with the torso; hold it hanging, more so the further the lean.
		var hang = leanNow > 0.02 ? Math.min(1.0, leanNow / posture.hangLean) : 0.0;
		var changed = false;
		for (control in limbs) if (control.apply(this, hang)) changed = true;
		if (changed) character.advance(0.0);
	}
}

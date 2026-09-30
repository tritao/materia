package humankit;

/** Animation and locomotion state shared by every action in a worker's job. */
class HumanBody {
	public final character:HumanCharacter;
	public final walker:HumanWalker;
	public final description:HumanDescription;
	/** True between a completed pick and its matching place. */
	public var grip(default, null):Bool = false;

	final active:Array<Bool> = [false, false, false, false];
	final modelTargets:Array<Bool> = [false, false, false, false];
	final targets:Array<Array<Float>> = [[], [], [], []];
	final weights:Array<Float> = [0.0, 0.0, 0.0, 0.0];
	final poles:Array<Null<Array<Float>>> = [null, null, null, null];
	final heldPoints:Array<Null<Void->Array<Float>>> = [null, null, null, null];
	var carrying:Array<HumanLimb> = [];
	/** Seconds a hand takes to settle into the carry pose from wherever it was. */
	static inline var CARRY_EASE_SECONDS:Float = 0.3;
	/** Where each hand was when it began to carry, and how far it has settled (one when done). */
	final carryFrom:Array<Null<Array<Float>>> = [null, null, null, null];
	final carrySettled:Array<Float> = [1.0, 1.0, 1.0, 1.0];

	public function new(character:HumanCharacter, ?walker:HumanWalker) {
		this.character = character;
		this.walker = walker == null ? new HumanWalker(character) : walker;
		if (this.walker.character != character)
			throw "A human body needs its character's walker";
		description = HumanDescription.measure(character.pose, character.height());
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
			?pole:Array<Float>):Void {
		var index:Int = limb;
		active[index] = true;
		modelTargets[index] = false;
		targets[index] = target.copy();
		weights[index] = Math.max(0.0, Math.min(1.0, weight));
		poles[index] = pole == null ? null : pole.copy();
	}

	/** Legacy model-space reach target used by HumanReachTask. */
	public function setReachModel(limb:HumanLimb, target:Array<Float>, weight:Float,
			?pole:Array<Float>):Void {
		var index:Int = limb;
		active[index] = true;
		modelTargets[index] = true;
		targets[index] = target.copy();
		weights[index] = Math.max(0.0, Math.min(1.0, weight));
		poles[index] = pole == null ? null : pole.copy();
	}

	public function reachWeight(limb:HumanLimb):Float
		return weights[limb];

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
		var limit = 0.95 * (description.upperArm + description.forearm);
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
		return poles[limb];

	public function reachTargetWorld(limb:HumanLimb):Null<Array<Float>> {
		var index:Int = limb;
		if (!active[index]) return null;
		return modelTargets[index] ? toWorld(targets[index]) : targets[index].copy();
	}

	public function clearReach(limb:HumanLimb):Void {
		var index:Int = limb;
		active[index] = false;
		weights[index] = 0.0;
		poles[index] = null;
		character.release(limb);
	}

	public function setCarry(hands:Array<HumanLimb>):Void {
		for (hand in carrying) if (hands.indexOf(hand) < 0) clearReach(hand);
		for (hand in hands) if (carrying.indexOf(hand) < 0) {
			// A hand takes up the carry pose from where it is, not by jumping there.
			var wrist = character.pose.bonePosition(hand == ArmL ? HandL : HandR);
			carryFrom[hand] = wrist == null ? null : wrist.copy();
			carrySettled[hand] = wrist == null ? 1.0 : 0.0;
		}
		carrying = hands.copy();
		for (hand in hands) clearReach(hand);
	}

	public function isCarrying(limb:HumanLimb):Bool
		return carrying.indexOf(limb) >= 0;

	/** Current carry target in model space, tied to the animated chest. */
	public function carryTargetModel(limb:HumanLimb):Array<Float> {
		var chest = character.pose.bonePosition(Chest);
		if (chest == null) chest = character.pose.bonePosition(Pelvis);
		if (chest == null) throw "The character has no chest or pelvis";
		var side = limb == ArmL ? 1.0 : -1.0;
		return [chest[0] + 0.24, chest[1] + side * 0.12, chest[2] - 0.22];
	}

	public function setGrip(held:Bool):Void
		grip = held;

	public function cancel():Void {
		walker.stop();
		carrying = [];
		grip = false;
		for (hand in [ArmL, ArmR]) setHeldPoint(hand, null);
		for (limb in [ArmL, ArmR, LegL, LegR]) clearReach(limb);
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
		for (hand in carrying) carrySettled[hand] = Math.min(1.0, carrySettled[hand] + seconds / CARRY_EASE_SECONDS);
		walker.advance(seconds);
		evaluate();
	}

	/** Re-evaluates reaches at the current animation time for in-frame IK solving. */
	public function evaluate():Void {
		var changed = false;
		for (limb in [ArmL, ArmR, LegL, LegR]) {
			var index:Int = limb;
			if (active[index]) {
				character.reach(limb, modelTargets[index] ? targets[index] : toModel(targets[index]),
					weights[index], poles[index]);
				changed = true;
			} else if (isCarrying(limb)) {
				var target = carryTargetModel(limb);
				var settled = carrySettled[index], from = carryFrom[index];
				if (settled < 1.0 && from != null) {
					var ease = settled * settled * (3.0 - 2.0 * settled);
					target = [for (axis in 0...3) from[axis] + (target[axis] - from[axis]) * ease];
				}
				character.reach(limb, target);
				changed = true;
			}
		}
		if (changed) character.advance(0.0);
	}
}

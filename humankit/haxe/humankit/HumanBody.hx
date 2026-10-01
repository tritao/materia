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
	/**
	 * How far the point a hand holds with (between the wrist and the knuckles) is beyond its wrist, in metres. The
	 * arm has to bring the wrist this much less far than the point it is reaching for.
	 */
	public final palmReach:Float;
	/** True between a completed pick and its matching place. */
	public var grip(default, null):Bool = false;

	/** The four limbs, indexed by HumanLimb. */
	final limbs:Array<LimbControl>;
	final heldPoints:Array<Null<Void->Array<Float>>> = [null, null, null, null];
	/** What each hand's fingers close to around what it holds, per finger; null for the plain grip. */
	final grasps:Array<Null<Array<Float>>> = [null, null, null, null];
	/** Whether each hand was holding something last step, so a grasp is dropped once what it held is let go. */
	final wasHolding:Array<Bool> = [false, false, false, false];
	var leanGoal:Float = 0.0;
	var leanNow:Float = 0.0;
	var hingeGoal:Float = 0.0;
	var hingeNow:Float = 0.0;
	/** Whether free arms are held down at the sides, as when walking up to a surface they must not sweep over. */
	var armsDown:Bool = false;
	/** Where each foot is held in the world (left, right), or null; and how strongly it is held. */
	final footAnchors:Array<Null<Array<Float>>> = [null, null];
	var footLock:Float = 0.0;
	final canLockFeet:Bool;
	var crouchGoal:Float = 0.0;
	var crouchNow:Float = 0.0;
	var kneelGoal:Float = 0.0;
	var kneelNow:Float = 0.0;
	/** The turn of the line between the shoulders when the body stands at rest, in radians; see bodyTurn. */
	var standingTurn:Float = 0.0;
	/** Where the bones planning reads stand when the body is upright and at rest, in model space. */
	final standing:Map<String, Array<Float>> = new Map();

	public function new(character:HumanCharacter, ?walker:HumanWalker, ?posture:HumanPosture) {
		this.character = character;
		this.walker = walker == null ? new HumanWalker(character) : walker;
		if (this.walker.character != character)
			throw "A human body needs its character's walker";
		description = HumanDescription.measure(character.pose, character.height());
		var wristAt = character.pose.bonePosition(HandL), knuckleAt = character.pose.bonePosition(MiddleL);
		palmReach = wristAt == null || knuckleAt == null ? 0.0 : 0.5 * Math.sqrt(Math.pow(knuckleAt[0] - wristAt[0], 2) +
			Math.pow(knuckleAt[1] - wristAt[1], 2) + Math.pow(knuckleAt[2] - wristAt[2], 2));
		this.posture = posture == null ? HumanPosture.forStature(description.stature) : posture;
		canLockFeet = character.legsAreChains();
		limbs = [for (limb in [ArmL, ArmR, LegL, LegR]) new LimbControl(limb, this.posture.relaxedCurl)];
		for (hand in [ArmL, ArmR]) character.setHandCurl(hand, this.posture.relaxedCurl);
		// Standing at rest is the idle pose: its shoulder line is what a turn is measured from.
		character.probe();
		standingTurn = shoulderTurn();
		for (bone in [HumanBone.UpperArmL, HumanBone.UpperArmR, HumanBone.Spine, HumanBone.Pelvis, HumanBone.ShinL, HumanBone.ShinR, HumanBone.FootL, HumanBone.FootR]) {
			var position = character.pose.bonePosition(bone);
			if (position != null) standing.set(bone, position.copy());
		}
	}

	/**
	 * Where a bone stands when the body is upright and at rest, in model space, for planning where to
	 * stand: the animated pose bobs with the gait, so reading it at the instant a plan is made would let
	 * a reach near the limit be possible in one phase of a stride and not another.
	 */
	public function standingBone(bone:HumanBone):Null<Array<Float>> {
		var position = standing.get(bone);
		return position == null ? null : position.copy();
	}

	/** What each finger is asked to close to: around what it holds, open while reaching, relaxed otherwise. */
	function curlsFor(hand:HumanLimb):Array<Float> {
		var control = limbs[hand];
		var holding = control.mode == Carry || heldPoints[hand] != null;
		if (holding) {
			wasHolding[hand] = true;
			var grasp = grasps[hand];
			return grasp != null ? grasp : uniformCurl(posture.gripCurl);
		}
		// What was held is let go: its shape goes with it.
		if (wasHolding[hand]) {
			wasHolding[hand] = false;
			grasps[hand] = null;
		}
		return uniformCurl(control.mode == Reach && control.weight > 0.0 ? posture.openCurl : posture.relaxedCurl);
	}

	static function uniformCurl(value:Float):Array<Float>
		return [for (_ in 0...HumanHand.FINGERS) value];

	/**
	 * How far an object reaches from the palm, straight out along the palm's normal, for the hand that
	 * is about to close on it: the fingers must curl down that far. `object` is the object's box.
	 */
	public function graspDepth(hand:HumanLimb, object:HumanTargetBox):Float {
		var normal = palmNormal(hand), root = rootTransform();
		var world = [root[0] * normal[0] + root[4] * normal[1] + root[8] * normal[2],
			root[1] * normal[0] + root[5] * normal[1] + root[9] * normal[2],
			root[2] * normal[0] + root[6] * normal[1] + root[10] * normal[2]];
		var c = Math.cos(object.yaw), s = Math.sin(object.yaw);
		return 2.0 * (Math.abs(world[0] * c + world[1] * s) * object.halfExtents[0] +
			Math.abs(-world[0] * s + world[1] * c) * object.halfExtents[1] + Math.abs(world[2]) * object.halfExtents[2]);
	}

	/** The direction the palm faces, in model space. The hand frame's Z leaves the two hands through opposite faces. */
	function palmNormal(hand:HumanLimb):Array<Float> {
		var frame = character.pose.boneFrame(hand == ArmL ? HandL : HandR);
		if (frame == null) throw "The character has no hand frame";
		var sign = hand == ArmL ? 1.0 : -1.0;
		return [sign * frame[8], sign * frame[9], sign * frame[10]];
	}

	/** Closes a hand's fingers around an object that reaches `depth` metres from the palm, and keeps them so while it holds it. */
	public function setGrasp(hand:HumanLimb, depth:Float):Void
		grasps[hand] = graspCurls(hand, depth);

	/** The curl each finger closes to around what a hand holds (THUMB to PINKY), or null when it holds nothing shaped. */
	public function grasp(hand:HumanLimb):Null<Array<Float>> {
		var curls = grasps[hand];
		return curls == null ? null : curls.copy();
	}

	/**
	 * How far a fingertip is from the palm along the palm's normal, in metres: positive on the palm's
	 * side, where an object is held. Zero for a hand without that finger.
	 */
	public function fingerDepth(hand:HumanLimb, finger:Int):Float {
		var tip = character.fingertip(hand, finger);
		var wrist = character.pose.bonePosition(hand == ArmL ? HandL : HandR);
		if (tip == null || wrist == null) return 0.0;
		var knuckle = character.pose.bonePosition(hand == ArmL ? MiddleL : MiddleR);
		var palm = knuckle == null ? wrist : [for (axis in 0...3) (wrist[axis] + knuckle[axis]) * 0.5];
		var normal = palmNormal(hand);
		return (tip[0] - palm[0]) * normal[0] + (tip[1] - palm[1]) * normal[1] + (tip[2] - palm[2]) * normal[2];
	}

	/**
	 * The curl each finger needs (THUMB to PINKY) to bring its tip to `depth` metres from the palm along
	 * its normal. Measured on the skeleton: the hand is curled in steps and each finger's tip depth
	 * recorded, so a short finger closes further than a long one to reach the same depth, and any rig
	 * works. A finger that never gets that deep closes as far as it gets; an object thinner than the
	 * posture's pinch limit is pinched between thumb and index. The pose is left as it was.
	 */
	function graspCurls(hand:HumanLimb, depth:Float):Array<Float> {
		var saved = character.handCurls(hand);
		var steps = posture.graspSteps;
		var depths:Array<Array<Float>> = [for (_ in 0...HumanHand.FINGERS) []];
		for (step in 0...steps + 1) {
			character.setHandCurl(hand, step / steps);
			character.probe();
			for (kind in 0...HumanHand.FINGERS) depths[kind].push(fingerDepth(hand, kind));
		}
		character.setHandCurls(hand, saved);
		character.probe();
		var curls = uniformCurl(0.0);
		for (kind in 1...HumanHand.FINGERS) {
			var reached = depths[kind], peak = 0, found = -1;
			for (step in 0...steps + 1) if (reached[step] > reached[peak]) peak = step;
			for (step in 0...steps + 1) if (reached[step] >= depth) { found = step; break; }
			// The first curl that brings the tip that deep; a finger that never gets there closes as far as it does.
			curls[kind] = found < 0 ? peak / steps : found == 0 ? 0.0 :
				(found - 1 + (depth - reached[found - 1]) / (reached[found] - reached[found - 1])) / steps;
		}
		if (depth < posture.pinchBelow)
			for (kind in [HumanHand.MIDDLE, HumanHand.RING, HumanHand.PINKY]) curls[kind] = Math.min(curls[kind], posture.relaxedCurl);
		curls[HumanHand.THUMB] = posture.thumbShare * curls[HumanHand.INDEX];
		return curls;
	}

	/** Where the torso is asked to lean; it eases there. Zero stands upright. */
	public function setLean(angle:Float):Void
		leanGoal = Math.max(0.0, Math.min(posture.maxLean, angle));

	/** Whether the character can crouch: its asset has a crouch clip to lower the body with. */
	public function canCrouch():Bool
		return character.canCrouch();

	/** Where the body is asked to crouch (0 stands, 1 is the clip's full crouch); it eases there. */
	public function setCrouch(amount:Float):Void
		crouchGoal = character.canCrouch() ? Math.max(0.0, Math.min(1.0, amount)) : 0.0;

	/** How far the body is into its crouch now. */
	public function crouchAmount():Float
		return crouchNow;

	/**
	 * Holds the arms that are not reaching or carrying down at the sides instead of letting the gait swing them,
	 * so walking up to a surface does not sweep a hand across what lies on it. Eases in and out like the hang a
	 * lean asks for. Pick and Place let go of it once their hand has the part.
	 */
	public function setArmsDown(down:Bool):Void
		armsDown = down;

	/** Whether the body has finished easing to the lean it was asked for. */
	public function leanReached():Bool
		return Math.abs(leanGoal - leanNow) < 1e-6 && Math.abs(hingeGoal - hingeNow) < 1e-6;

	/** Where the body is asked to bend at the hips, on top of its lean; it eases there. Zero stands straight. */
	public function setHinge(angle:Float):Void
		hingeGoal = Math.max(0.0, Math.min(posture.maxHinge, angle));

	/** Whether the body has finished easing to the crouch it was asked for. */
	public function crouchReached():Bool
		return Math.abs(crouchGoal - crouchNow) < 1e-6;

	/** Whether the character can kneel: its asset has a kneeling clip to lower the body onto a knee with. */
	public function canKneel():Bool
		return character.canKneel();

	/**
	 * Where the body is asked to kneel (0 stands, 1 the lowest the kneeling clip goes); it eases there. A kneel and a crouch are
	 * different ways down, so a body kneels only from standing: ask for a crouch of nothing first.
	 */
	public function setKneel(amount:Float):Void
		kneelGoal = character.canKneel() ? Math.max(0.0, Math.min(1.0, amount)) : 0.0;

	/** How far the body is into its kneel now. */
	public function kneelAmount():Float
		return kneelNow;

	public function kneelReached():Bool
		return Math.abs(kneelGoal - kneelNow) < 1e-6;

	/** Whether the body has finished easing to the crouch and the kneel it was asked for. */
	public function downReached():Bool
		return crouchReached() && kneelReached();

	/** How far down, in either way, the body is now: 0 standing. */
	public function downAmount():Float
		return Math.max(crouchNow, kneelNow);

	/**
	 * How far a bone sits from where it stands upright when the body crouches `amount`, in model space.
	 * Measured on the skeleton at the current animation time, with no lean; the pose is left as it was.
	 */
	public function crouchShift(bone:HumanBone, amount:Float):Array<Float>
		return downShift(bone, amount, 0.0);

	/** The same for a kneel: how far a bone sits from standing when the body kneels `amount`. */
	public function kneelShift(bone:HumanBone, amount:Float):Array<Float>
		return downShift(bone, 0.0, amount);

	function downShift(bone:HumanBone, crouchAmount:Float, kneelAmount:Float):Array<Float> {
		if ((crouchAmount <= 0.0 && kneelAmount <= 0.0) || (!character.canCrouch() && !character.canKneel())) return [0.0, 0.0, 0.0];
		var savedCrouch = character.crouch(), savedKneel = character.kneel(), savedLean = character.spineLean();
		character.setSpineLean(0.0);
		character.setDown(0.0, 0.0);
		character.probe();
		var upright = character.pose.bonePosition(bone);
		character.setDown(crouchAmount, kneelAmount);
		character.probe();
		var down = character.pose.bonePosition(bone);
		character.setDown(savedCrouch, savedKneel);
		character.setSpineLean(savedLean);
		character.probe();
		if (upright == null || down == null) return [0.0, 0.0, 0.0];
		return [down[0] - upright[0], down[1] - upright[1], down[2] - upright[2]];
	}

	/**
	 * The lean that carries a hand's shoulder `shift` metres further forward, and the shift it can
	 * deliver (less than asked when that would pass the posture's maximum lean). Measured by leaning
	 * the skeleton a test amount, so it holds for any rig; the pose is left as it was. `crouch` is the
	 * depth the body is planned to be at, since a crouched torso carries the shoulder differently.
	 */
	public function leanFor(hand:HumanLimb, shift:Float, crouch:Float = 0.0, kneel:Float = 0.0):{angle:Float, shift:Float, drop:Float}
		return pitchFor(hand, shift, crouch, kneel, false, 0.0);

	/**
	 * The same for bending at the hips, on top of a lean already planned (`lean`): the hinge that carries the shoulder `shift`
	 * metres further forward than that lean does, and the shift and drop it delivers, up to the posture's maximum hinge.
	 */
	public function hingeFor(hand:HumanLimb, shift:Float, crouch:Float, kneel:Float, lean:Float):{angle:Float, shift:Float, drop:Float}
		return pitchFor(hand, shift, crouch, kneel, true, lean);

	function pitchFor(hand:HumanLimb, shift:Float, crouch:Float, kneel:Float, hinging:Bool, base:Float):{angle:Float, shift:Float, drop:Float} {
		var none = {angle: 0.0, shift: 0.0, drop: 0.0};
		if (!(shift > 1e-4)) return none;
		var bone = hand == ArmL ? HumanBone.UpperArmL : HumanBone.UpperArmR;
		var saved = character.spineLean(), savedHinge = character.spineHinge(), savedCrouch = character.crouch(), savedKneel = character.kneel();
		var limit = hinging ? posture.maxHinge : posture.maxLean;
		character.setDown(crouch, kneel);
		character.setSpineLean(hinging ? base : 0.0);
		character.setSpineHinge(0.0);
		character.probe();
		var upright = character.pose.bonePosition(bone);
		var at = function(angle:Float):Null<Array<Float>> {
			if (hinging) character.setSpineHinge(angle);
			else character.setSpineLean(angle);
			character.probe();
			return character.pose.bonePosition(bone);
		};
		// The forward shift is not linear in the angle, so the probe only gives a first guess: the angle is then
		// corrected where it is used, and the shift and the drop of the shoulder reported are the measured ones.
		var probe = 0.2;
		var leaned = at(probe);
		var result = none;
		if (upright != null && leaned != null && leaned[0] - upright[0] > 1e-4) {
			var angle = Math.min(limit, shift * probe / (leaned[0] - upright[0]));
			for (round in 0...3) {
				var there = at(angle);
				if (there == null) break;
				var made = there[0] - upright[0];
				result = {angle: angle, shift: made, drop: there[2] - upright[2]};
				if (Math.abs(made - shift) < 5e-4 || angle >= limit - 1e-6 && made < shift) break;
				angle = Math.min(limit, angle * shift / Math.max(1e-4, made));
			}
		}
		character.setSpineLean(saved);
		character.setSpineHinge(savedHinge);
		character.setDown(savedCrouch, savedKneel);
		character.probe();
		return result;
	}

	function moveHinge(seconds:Float):Void {
		var step = posture.leanRate * seconds;
		hingeNow = Math.abs(hingeGoal - hingeNow) <= step ? hingeGoal : hingeNow + (hingeGoal > hingeNow ? step : -step);
		character.setSpineHinge(hingeNow);
	}

	function moveLean(seconds:Float):Void {
		var step = posture.leanRate * seconds;
		leanNow = Math.abs(leanGoal - leanNow) <= step ? leanGoal : leanNow + (leanGoal > leanNow ? step : -step);
		character.setSpineLean(leanNow);
	}

	/**
	 * While the idle pose shows (standing, and the fade into and out of a walk), the feet are held where they stand:
	 * the idle pose keeps its feet still relative to the body, so a body that starts to move drags them, and one that
	 * lowers or rises slides them. Each foot is reached for at the spot it had when the hold began, with a hold that
	 * comes in and goes out smoothly; in a steady walk the gait already keeps a planted foot still.
	 */
	function holdFeet(seconds:Float):Void {
		if (!canLockFeet || !posture.lockFeet) return;
		// A kneel puts a foot behind the body and a knee on the floor, which a foot held where it stood would fight.
		var share = kneelNow > 0.01 || kneelGoal > 0.0 ? 0.0 : walker.stanceShare();
		var wanted = Math.max(0.0, Math.min(1.0, (share - posture.lockFrom) / posture.lockSpan));
		wanted = wanted * wanted * (3.0 - 2.0 * wanted);
		var step = seconds / posture.lockSeconds;
		footLock = Math.abs(wanted - footLock) <= step ? wanted : footLock + (wanted > footLock ? step : -step);
		var feet = [HumanBone.FootL, HumanBone.FootR], legs = [LegL, LegR];
		if (footLock <= 1e-4) {
			for (side in 0...2) if (footAnchors[side] != null) {
				footAnchors[side] = null;
				clearReach(legs[side]);
			}
			return;
		}
		for (side in 0...2) {
			var foot = character.pose.bonePosition(feet[side]);
			if (foot == null) continue;
			var here = toWorld(foot);
			var anchor = footAnchors[side];
			// Taken where the foot is now, as the hold begins; let go and taken again if the body has left it too far behind.
			if (anchor == null || distance(anchor, here) > posture.lockReach + 0.5 * (1.0 - footLock)) {
				anchor = here;
				footAnchors[side] = anchor;
			}
			setReachWorld(legs[side], anchor, footLock);
		}
	}

	function moveKneel(seconds:Float):Void {
		var step = posture.kneelRate * seconds;
		kneelNow = Math.abs(kneelGoal - kneelNow) <= step ? kneelGoal : kneelNow + (kneelGoal > kneelNow ? step : -step);
		if (character.canKneel() && Math.abs(character.kneel() - kneelNow) > 1e-9) character.setKneel(kneelNow);
	}

	function moveCrouch(seconds:Float):Void {
		var step = posture.crouchRate * seconds;
		crouchNow = Math.abs(crouchGoal - crouchNow) <= step ? crouchGoal : crouchNow + (crouchGoal > crouchNow ? step : -step);
		if (character.canCrouch() && Math.abs(character.crouch() - crouchNow) > 1e-9) character.setCrouch(crouchNow);
	}

	function moveFingers(seconds:Float):Void {
		for (hand in [ArmL, ArmR]) {
			var control = limbs[hand];
			var step = posture.curlRate * seconds, wanted = curlsFor(hand);
			for (kind in 0...HumanHand.FINGERS)
				control.curls[kind] = Math.abs(wanted[kind] - control.curls[kind]) <= step ? wanted[kind] :
					control.curls[kind] + (wanted[kind] > control.curls[kind] ? step : -step);
			character.setHandCurls(hand, control.curls);
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
		limbs[limb].reach(target, ReachSpace.World, weight, pole);

	/** Legacy model-space reach target used by HumanReachTask. */
	public function setReachModel(limb:HumanLimb, target:Array<Float>, weight:Float,
			?pole:Array<Float>):Void
		limbs[limb].reach(target, ReachSpace.Model, weight, pole);

	/**
	 * Reaches for a point given as an offset from the chest (model-space axes), so the target moves with the
	 * torso: it keeps its place against the shoulder as the body leans, straightens, and walks.
	 */
	public function setReachChest(limb:HumanLimb, offset:Array<Float>, weight:Float, ?pole:Array<Float>):Void
		limbs[limb].reach(offset, ReachSpace.Torso, weight, pole);

	/** Where the chest is, in model space (the pelvis on a rig without one). */
	public function chestPosition():Array<Float> {
		var chest = character.pose.bonePosition(Chest);
		if (chest == null) chest = character.pose.bonePosition(Pelvis);
		if (chest == null) throw "The character has no chest or pelvis";
		return chest;
	}

	/** A limb's reach target in model space, whichever frame it was given in. */
	public function reachTargetModel(control:LimbControl):Array<Float> {
		if (control.space == ReachSpace.World) return toModel(control.target);
		if (control.space == ReachSpace.Model) return control.target.copy();
		var chest = chestPosition();
		return [for (axis in 0...3) chest[axis] + control.target[axis]];
	}

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
		// The guess is the point to be held at; the wrist, which the arm has to bring there, starts a palm's depth short of it.
		var shoulderAt = character.pose.bonePosition(hand == ArmL ? UpperArmL : UpperArmR);
		if (shoulderAt != null && palmReach > 0.0) {
			var from = toWorld(shoulderAt);
			var gap = distance(from, guess);
			if (gap > palmReach * 2.0)
				guess = [for (axis in 0...3) guess[axis] - (guess[axis] - from[axis]) / gap * palmReach];
		}
		var failure = reachFailure(hand, guess);
		if (failure != null) return {goal: guess.copy(), error: Math.POSITIVE_INFINITY, failure: failure};
		var best = guess.copy(), bestMiss = [0.0, 0.0, 0.0], bestError = Math.POSITIVE_INFINITY;
		var goal = guess.copy(), gain = 1.0;
		for (attempt in 0...8) {
			if (reachFailure(hand, goal) == null) {
				setReachWorld(hand, goal, 1.0);
				evaluate(false);
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

	/**
	 * How far a limb's wrist would travel, in metres, if its reach were let go now: the distance from where
	 * the IK holds it to where its animation puts it. The pose is left as it was.
	 */
	public function travelToAnimation(limb:HumanLimb):Float {
		var bone = limb == ArmL ? HumanBone.HandL : limb == ArmR ? HumanBone.HandR : null;
		if (bone == null) return 0.0;
		var held = character.pose.bonePosition(bone);
		if (held == null) return 0.0;
		character.release(limb);
		character.probe();
		var free = character.pose.bonePosition(bone);
		// The limb's reach is applied again, from its own record of what it is asked to do.
		evaluate();
		return free == null ? 0.0 : distance(held, free);
	}

	/** Where a hand's wrist is in the world now. */
	public function wristWorld(limb:HumanLimb):Array<Float> {
		var wrist = character.pose.bonePosition(limb == ArmL ? HandL : HandR);
		if (wrist == null) throw "The character has no hand bone";
		return toWorld(wrist);
	}

	/**
	 * How long a reach takes to blend in from the animation or out to it, given how far the wrist has to go: at
	 * least `minimum`, and longer when the distance would otherwise push the wrist past the posture's blend speed.
	 * Easing peaks at half as much again as the average speed, hence the factor.
	 */
	public function blendSeconds(distance:Float, minimum:Float):Float
		return Math.max(minimum, 1.5 * distance / posture.blendSpeed);

	public function reachPole(limb:HumanLimb):Null<Array<Float>>
		return limbs[limb].pole;

	public function reachTargetWorld(limb:HumanLimb):Null<Array<Float>> {
		var control = limbs[limb];
		if (control.mode != Reach) return null;
		return control.space == ReachSpace.World ? control.target.copy() : toWorld(reachTargetModel(control));
	}

	public function clearReach(limb:HumanLimb):Void
		limbs[limb].release(character);

	public function setCarry(hands:Array<HumanLimb>):Void {
		walker.setCarrying(hands.length > 0);
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
		var chest = chestPosition();
		var side = limb == ArmL ? 1.0 : -1.0;
		// A load is held in front of the body, which a turn clip turns in model space without moving the root: the offset turns
		// with it, or the hands would stay where the root faces and jump to the body when the root takes up the turn.
		var turn = walker.turningByClip() ? bodyTurn() : 0.0, c = Math.cos(turn), s = Math.sin(turn);
		var ahead = posture.carryOffset[0], across = side * posture.carryOffset[1];
		return [chest[0] + ahead * c - across * s, chest[1] + ahead * s + across * c, chest[2] + posture.carryOffset[2]];
	}

	function shoulderTurn():Float {
		var left = character.pose.bonePosition(UpperArmL), right = character.pose.bonePosition(UpperArmR);
		return left == null || right == null ? 0.0 : Math.atan2(left[1] - right[1], left[0] - right[0]);
	}

	/**
	 * How far the idle pose's shoulders are twisted from square to the way the body faces, in radians (positive: the left shoulder
	 * further back). A body that faces a table with its shoulders square needs its root turned by the opposite.
	 */
	public function standingTwist():Float {
		var twist = standingTurn - Math.PI * 0.5;
		while (twist > Math.PI) twist -= 2.0 * Math.PI;
		while (twist < -Math.PI) twist += 2.0 * Math.PI;
		return Math.abs(twist) < 0.05 ? 0.0 : twist;
	}

	/** How far the body is turned in model space from standing at rest (a turn clip turns it so), in radians. */
	public function bodyTurn():Float {
		var turn = shoulderTurn() - standingTurn;
		while (turn > Math.PI) turn -= 2.0 * Math.PI;
		while (turn < -Math.PI) turn += 2.0 * Math.PI;
		return turn;
	}

	public function setGrip(held:Bool):Void
		grip = held;

	public function cancel():Void {
		walker.stop();
		walker.setCarrying(false);
		grip = false;
		for (hand in [ArmL, ArmR]) setHeldPoint(hand, null);
		for (control in limbs) control.release(character);
		for (hand in [ArmL, ArmR]) {
			for (kind in 0...HumanHand.FINGERS) limbs[hand].curls[kind] = posture.relaxedCurl;
			character.setHandCurl(hand, posture.relaxedCurl);
			grasps[hand] = null;
			wasHolding[hand] = false;
		}
		leanGoal = 0.0;
		leanNow = 0.0;
		hingeGoal = 0.0;
		hingeNow = 0.0;
		character.setSpineHinge(0.0);
		character.setSpineLean(0.0);
		armsDown = false;
		footLock = 0.0;
		footAnchors[0] = null;
		footAnchors[1] = null;
		crouchGoal = 0.0;
		crouchNow = 0.0;
		kneelGoal = 0.0;
		kneelNow = 0.0;
		character.setDown(0.0, 0.0);
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
		// A lean swings an unused arm back with the torso, so while the body leans a free arm is held hanging.
		for (control in limbs) control.advance(seconds, posture.carryEaseSeconds, leanGoal > 0.02 || leanNow > 0.02 || hingeNow > 0.02 || hingeGoal > 0.02 || crouchNow > 0.02 || kneelNow > 0.02 || armsDown, posture.hangSeconds);
		moveFingers(seconds);
		moveLean(seconds);
		moveHinge(seconds);
		moveCrouch(seconds);
		moveKneel(seconds);
		walker.advance(seconds);
		holdFeet(seconds);
		evaluate();
	}

	/**
	 * Re-evaluates reaches at the current animation time for in-frame IK solving. With `present` off the pose is
	 * only measured (joint matrices current, nothing skinned or shown), for the attempts of a solve that only the
	 * last of which is to be seen.
	 */
	public function evaluate(present:Bool = true):Void {
		var changed = false;
		for (control in limbs) if (control.apply(this)) changed = true;
		if (!changed) return;
		if (present) character.advance(0.0);
		else character.probe();
	}
}

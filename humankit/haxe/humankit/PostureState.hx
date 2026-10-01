package humankit;

/**
 * How far a body is leaned, bent at the hips, crouched and kneeling, and what it has been asked for. Each is a goal
 * the body eases toward at the rate `HumanPosture` sets, written to the character as it goes. A body that cannot
 * crouch or kneel (its asset has no clip for it) ignores the ask.
 */
class PostureState {
	final character:HumanCharacter;
	final posture:HumanPosture;
	var leanGoal:Float = 0.0;
	var leanNow:Float = 0.0;
	var hingeGoal:Float = 0.0;
	var hingeNow:Float = 0.0;
	var crouchGoal:Float = 0.0;
	var crouchNow:Float = 0.0;
	var kneelGoal:Float = 0.0;
	var kneelNow:Float = 0.0;

	public function new(character:HumanCharacter, posture:HumanPosture) {
		this.character = character;
		this.posture = posture;
	}

	/** Where the torso is asked to lean; it eases there. Zero stands upright. */
	public function setLean(angle:Float):Void
		leanGoal = Math.max(0.0, Math.min(posture.maxLean, angle));

	/** Where the body is asked to bend at the hips, on top of its lean; it eases there. Zero stands straight. */
	public function setHinge(angle:Float):Void
		hingeGoal = Math.max(0.0, Math.min(posture.maxHinge, angle));

	/** Where the body is asked to crouch (0 stands, 1 is the clip's full crouch); it eases there. */
	public function setCrouch(amount:Float):Void
		crouchGoal = character.canCrouch() ? Math.max(0.0, Math.min(1.0, amount)) : 0.0;

	/**
	 * Where the body is asked to kneel (0 stands, 1 the lowest the kneeling clip goes); it eases there. A kneel and a crouch are
	 * different ways down, so a body kneels only from standing: ask for a crouch of nothing first.
	 */
	public function setKneel(amount:Float):Void
		kneelGoal = character.canKneel() ? Math.max(0.0, Math.min(1.0, amount)) : 0.0;

	/** How far the body is into its crouch now. */
	public function crouchAmount():Float
		return crouchNow;

	/** How far the body is into its kneel now. */
	public function kneelAmount():Float
		return kneelNow;

	/** How far down, in either way, the body is now: 0 standing. */
	public function downAmount():Float
		return Math.max(crouchNow, kneelNow);

	/** Whether the body has finished easing to the lean and the hinge it was asked for. */
	public function leanReached():Bool
		return Math.abs(leanGoal - leanNow) < 1e-6 && Math.abs(hingeGoal - hingeNow) < 1e-6;

	public function crouchReached():Bool
		return Math.abs(crouchGoal - crouchNow) < 1e-6;

	public function kneelReached():Bool
		return Math.abs(kneelGoal - kneelNow) < 1e-6;

	/** Whether the body has finished easing to the crouch and the kneel it was asked for. */
	public function downReached():Bool
		return crouchReached() && kneelReached();

	/** Whether the body is, or is on its way to being, leaned, bent or down: the cue for holding a free arm hanging. */
	public function isBent():Bool
		return leanGoal > 0.02 || leanNow > 0.02 || hingeNow > 0.02 || hingeGoal > 0.02 || crouchNow > 0.02 || kneelNow > 0.02;

	/** Whether the body is on a knee or asked to be: a foot cannot be held where it stood then. */
	public function isKneeling():Bool
		return kneelNow > 0.01 || kneelGoal > 0.0;

	/** How far the upper body is pitched forward now, in radians: the lean and the hip hinge together. */
	public function torsoPitch():Float
		return character.spineLean() + character.spineHinge();

	/** Eases each toward its goal over `seconds` and writes the result to the character. */
	public function advance(seconds:Float):Void {
		moveSpine(seconds);
		moveCrouch(seconds);
		moveKneel(seconds);
	}

	/** Stands the body upright at once, with nothing asked for. */
	public function reset():Void {
		leanGoal = 0.0;
		leanNow = 0.0;
		hingeGoal = 0.0;
		hingeNow = 0.0;
		crouchGoal = 0.0;
		crouchNow = 0.0;
		kneelGoal = 0.0;
		kneelNow = 0.0;
		character.setSpineHinge(0.0);
		character.setSpineLean(0.0);
		character.setDown(0.0, 0.0);
	}

	/**
	 * Lean and hinge both pitch the torso, so they share one rate: moving both at the full rate would pitch the torso
	 * at twice the pace the posture sets, and carry the shoulder, and the hand with it, past the limits on hand speed.
	 * Each moves the same share of what it has to go, so the two arrive together.
	 */
	function moveSpine(seconds:Float):Void {
		var leanGap = leanGoal - leanNow, hingeGap = hingeGoal - hingeNow;
		var total = Math.abs(leanGap) + Math.abs(hingeGap);
		var budget = posture.leanRate * seconds;
		var share = total <= budget || total <= 0.0 ? 1.0 : budget / total;
		leanNow = share >= 1.0 ? leanGoal : leanNow + leanGap * share;
		hingeNow = share >= 1.0 ? hingeGoal : hingeNow + hingeGap * share;
		character.setSpineLean(leanNow);
		character.setSpineHinge(hingeNow);
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
}

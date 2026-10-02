package humankit;

import humankit.action.Carry;
import humankit.action.Reach;

/**
 * One limb of a body: what it was asked to do, and what its IK holds now. The owner sets the asked
 * mode (reach, carry, or release); `apply` then reconciles the character with it, releasing IK only
 * when the character still holds something the limb no longer wants. `applied` is the single record of
 * what the character holds, so there is no flag that can go stale.
 */
class LimbControl {
	public final limb:HumanLimb;
	public var mode(default, null):LimbMode = Free;
	/** The reach target, and the frame it is given in. */
	public var target(default, null):Array<Float> = [];
	public var space(default, null):ReachSpace = ReachSpace.World;
	public var weight(default, null):Float = 0.0;
	public var pole(default, null):Null<Array<Float>> = null;
	/** The curl of each of this hand's fingers now (HumanHand.THUMB to PINKY); they move toward what the limb's state asks for. */
	public final curls:Array<Float>;
	/** Where the wrist was when carrying began, and how far it has settled into the carry pose (one when done). */
	var carryFrom:Null<Array<Float>> = null;
	var carrySettled:Float = 1.0;
	/** What the character's IK holds for this limb now. */
	var applied:LimbMode = Free;
	/** How far a free arm has settled into hanging, 0 to 1; it eases in and out over the posture's hang time. */
	var hang:Float = 0.0;

	public function new(limb:HumanLimb, curl:Float) {
		this.limb = limb;
		curls = [for (_ in 0...HumanHand.FINGERS) curl];
	}

	public function isArm():Bool
		return limb == ArmL || limb == ArmR;

	public function reach(target:Array<Float>, space:ReachSpace, weight:Float, ?pole:Array<Float>):Void {
		mode = Reach;
		this.space = space;
		this.target = target.copy();
		this.weight = Math.max(0.0, Math.min(1.0, weight));
		this.pole = pole == null ? null : pole.copy();
	}

	/** Takes up the carry pose, easing in from `from`, the wrist's position now, when there is one. */
	public function carry(from:Null<Array<Float>>):Void {
		if (mode == Carry) return;
		mode = Carry;
		weight = 0.0;
		pole = null;
		carryFrom = from == null ? null : from.copy();
		carrySettled = from == null ? 1.0 : 0.0;
	}

	/** Goes back to the animation, releasing the character's IK on the spot. */
	public function release(character:HumanCharacter):Void {
		mode = Free;
		weight = 0.0;
		pole = null;
		dropIk(character);
	}

	/** Releases the character's IK for this limb without changing what the limb is asked to do. */
	public function dropIk(character:HumanCharacter):Void {
		character.release(limb);
		applied = Free;
	}

	public function advance(seconds:Float, carryEaseSeconds:Float, hangWanted:Bool, hangSeconds:Float):Void {
		if (mode == Carry) carrySettled = Math.min(1.0, carrySettled + seconds / carryEaseSeconds);
		// Only a free arm hangs; one that reaches or carries starts from not hanging next time.
		var step = seconds / hangSeconds;
		if (mode != Free || !isArm()) hang = 0.0;
		else hang = hangWanted ? Math.min(1.0, hang + step) : Math.max(0.0, hang - step);
	}

	/**
	 * Brings the character's IK for this limb in line with what it is asked to do; returns whether the
	 * pose changed.
	 */
	public function apply(body:HumanBody):Bool {
		var character = body.character;
		var wanted = mode == Free && isArm() && hang > 0.0 ? Hang : mode;
		switch wanted {
			case Reach:
				character.reach(limb, body.reachTargetModel(this), weight, pole);
			case Carry:
				var goal = body.carryTargetModel(limb);
				var from = carryFrom;
				if (carrySettled < 1.0 && from != null) {
					var ease = carrySettled * carrySettled * (3.0 - 2.0 * carrySettled);
					goal = [for (axis in 0...3) from[axis] + (goal[axis] - from[axis]) * ease];
				}
				character.reach(limb, goal);
			case Hang:
				// An arm the lean is not using would swing back with the torso: hang it under the shoulder.
				var shoulder = character.pose.bonePosition(limb == ArmL ? UpperArmL : UpperArmR);
				if (shoulder == null) return false;
				var posture = body.posture;
				var side = limb == ArmL ? 1.0 : -1.0;
				var length = body.description.upperArm + body.description.forearm;
				character.reach(limb, [shoulder[0] + posture.hangForward, shoulder[1] + side * posture.hangOutward,
					shoulder[2] - posture.hangDrop * length], hang * hang * (3.0 - 2.0 * hang));
			case Free:
				if (applied == Free) return false;
				character.release(limb);
		}
		applied = wanted;
		return true;
	}
}

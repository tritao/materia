package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;

/**
 * One hand's fingers, each curled between open (0) and a fist (1). A finger is each child of the
 * wrist and the single-child chain below it, found from the skeleton so any humanoid rig works. Every
 * joint turns about its own local X axis, negatively, which brings the fingertip toward the palm on
 * both hands of the rigs HumanKit maps. The turns ride on the animation, so fingers keep whatever
 * their clip did and only add the curl.
 */
class HumanHand {
	/** The fingers, in the order curls are given. */
	public static inline var THUMB:Int = 0;
	public static inline var INDEX:Int = 1;
	public static inline var MIDDLE:Int = 2;
	public static inline var RING:Int = 3;
	public static inline var PINKY:Int = 4;
	public static inline var FINGERS:Int = 5;

	/** Largest turn of each segment of a finger, knuckle outward, in radians. */
	static final FINGER_ANGLES:Array<Float> = [1.35, 1.6, 1.1];
	static final THUMB_ANGLES:Array<Float> = [0.4, 0.6, 0.6];

	final instance:AnimationInstance;
	/** The joint-turn source the curl is applied under, so it never overwrites another feature's turns. */
	final source:Int;
	/** Each finger's kind, its curling joints with the largest turn of each, and its last joint (the tip). */
	final fingers:Array<{kind:Int, segments:Array<{joint:Int, angle:Float}>, tip:Int}> = [];
	/** How curled each finger is now (by kind), or -1 before the first curl. */
	final curls:Array<Float> = [for (_ in 0...FINGERS) -1.0];
	/** How curled the hand is now on average, or -1 before the first curl. */
	public var curl(default, null):Float = -1.0;

	function new(instance:AnimationInstance, source:Int) {
		this.instance = instance;
		this.source = source;
	}

	/** The fingers below a rig's wrist bone, or null when the skeleton has none. */
	public static function find(asset:AnimationAsset, rig:HumanoidRig, instance:AnimationInstance, wrist:HumanBone,
			source:Int):Null<HumanHand> {
		var root = rig.joint(wrist);
		if (root < 0) return null;
		var hand = new HumanHand(instance, source);
		for (first in children(asset, root)) {
			var chain = [first];
			while (true) {
				var next = children(asset, chain[chain.length - 1]);
				if (next.length != 1) break;
				chain.push(next[0]);
			}
			var kind = kindOf(asset.jointNames[first]);
			if (kind < 0) continue;
			var thumb = kind == THUMB;
			var angles = thumb ? THUMB_ANGLES : FINGER_ANGLES;
			// Quaternius's first finger joints share one pivot in the palm and do not bend.
			if (!thumb && rig.mapping.name == "quaternius" && chain.length >= 4) chain.shift();
			var curling:Array<{joint:Int, angle:Float}> = [];
			for (index in 0...chain.length) {
				var angle = index < angles.length ? angles[index] : angles[angles.length - 1] * 0.7;
				curling.push({joint: chain[index], angle: angle});
			}
			hand.fingers.push({kind: kind, segments: curling, tip: chain[chain.length - 1]});
		}
		return hand.fingers.length == 0 ? null : hand;
	}

	/** Which finger a joint name belongs to, or -1. */
	static function kindOf(jointName:String):Int {
		var name = jointName.toLowerCase();
		if (name.indexOf("thumb") >= 0) return THUMB;
		if (name.indexOf("index") >= 0) return INDEX;
		if (name.indexOf("middle") >= 0) return MIDDLE;
		if (name.indexOf("ring") >= 0) return RING;
		if (name.indexOf("pinky") >= 0 || name.indexOf("little") >= 0) return PINKY;
		return -1;
	}

	/** Whether the hand has this finger. */
	public function has(kind:Int):Bool {
		for (finger in fingers) if (finger.kind == kind) return true;
		return false;
	}

	/** How curled a finger is now, or 0 before the first curl. */
	public function curlOf(kind:Int):Float
		return curls[kind] < 0.0 ? 0.0 : curls[kind];

	/** The model-space position of a finger's last joint, or null when the hand lacks the finger. */
	public function tipPosition(kind:Int):Null<Array<Float>> {
		for (finger in fingers) if (finger.kind == kind) {
			var matrices = instance.readJointMatrices(), base = finger.tip * 64;
			return [matrices.getFloat(base + 48), matrices.getFloat(base + 52), matrices.getFloat(base + 56)];
		}
		return null;
	}

	/** Joints whose parent is `parent`, leaving out the empty tip markers a rig exports ("_end"). */
	static function children(asset:AnimationAsset, parent:Int):Array<Int> {
		var found:Array<Int> = [];
		for (index in 0...asset.jointParents.length) {
			if (asset.jointParents[index] != parent) continue;
			var name = asset.jointNames[index].toLowerCase();
			if (name.substr(name.length - 4) == "_end") continue;
			found.push(index);
		}
		return found;
	}

	/** Curls every finger alike, 0 open to 1 a fist. */
	public function setCurl(value:Float):Void
		setCurls([for (_ in 0...FINGERS) value]);

	/**
	 * Curls each finger on its own: values are indexed THUMB to PINKY, each 0 open to 1 a fist. Fingers
	 * the hand lacks are ignored.
	 */
	public function setCurls(values:Array<Float>):Void {
		var clamped = [for (kind in 0...FINGERS) Math.max(0.0, Math.min(1.0, values[kind]))];
		var changed = false;
		for (kind in 0...FINGERS) if (Math.abs(clamped[kind] - curls[kind]) >= 1e-4) changed = true;
		if (!changed) return;
		var total = 0.0, count = 0;
		for (finger in fingers) {
			curls[finger.kind] = clamped[finger.kind];
			total += clamped[finger.kind];
			count++;
		}
		curl = count == 0 ? 0.0 : total / count;
		var joints:Array<Int> = [], rotations:Array<Array<Float>> = [], weights:Array<Float> = [];
		for (finger in fingers) for (segment in finger.segments) {
			var turn = segment.angle * clamped[finger.kind];
			if (turn < 1e-4) continue;
			joints.push(segment.joint);
			rotations.push([-Math.sin(turn * 0.5), 0.0, 0.0, Math.cos(turn * 0.5)]);
			weights.push(1.0);
		}
		// One call replaces the whole hand's turns.
		if (joints.length == 0) instance.clearJointRotations(source);
		else instance.setJointRotations(source, joints, rotations, weights);
	}
}

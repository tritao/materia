package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;

/**
 * One hand's fingers, curled together between open (0) and a fist (1). A finger is each child of the
 * wrist and the single-child chain below it, found from the skeleton so any humanoid rig works. Every
 * joint turns about its own local X axis, negatively, which brings the fingertip toward the palm on
 * both hands of the rigs HumanKit maps. The turns ride on the animation, so fingers keep whatever
 * their clip did and only add the curl.
 */
class HumanHand {
	/** Largest turn of each segment of a finger, knuckle outward, in radians. */
	static final FINGER_ANGLES:Array<Float> = [1.35, 1.6, 1.1];
	static final THUMB_ANGLES:Array<Float> = [0.4, 0.6, 0.6];

	final instance:AnimationInstance;
	/** Each finger's curling joints with the largest turn of each. */
	final fingers:Array<Array<{joint:Int, angle:Float}>> = [];
	/** How curled the hand is now, or -1 before the first curl. */
	public var curl(default, null):Float = -1.0;

	function new(instance:AnimationInstance) {
		this.instance = instance;
	}

	/** The fingers below a rig's wrist bone, or null when the skeleton has none. */
	public static function find(asset:AnimationAsset, rig:HumanoidRig, instance:AnimationInstance,
			wrist:HumanBone):Null<HumanHand> {
		var root = rig.joint(wrist);
		if (root < 0) return null;
		var hand = new HumanHand(instance);
		for (first in children(asset, root)) {
			var chain = [first];
			while (true) {
				var next = children(asset, chain[chain.length - 1]);
				if (next.length != 1) break;
				chain.push(next[0]);
			}
			var thumb = asset.jointNames[first].toLowerCase().indexOf("thumb") >= 0;
			var angles = thumb ? THUMB_ANGLES : FINGER_ANGLES;
			// Quaternius's first finger joints share one pivot in the palm and do not bend.
			if (!thumb && rig.mapping.name == "quaternius" && chain.length >= 4) chain.shift();
			var curling:Array<{joint:Int, angle:Float}> = [];
			for (index in 0...chain.length) {
				var angle = index < angles.length ? angles[index] : angles[angles.length - 1] * 0.7;
				curling.push({joint: chain[index], angle: angle});
			}
			hand.fingers.push(curling);
		}
		return hand.fingers.length == 0 ? null : hand;
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

	/** Curls every finger, 0 open to 1 a fist. */
	public function setCurl(value:Float):Void {
		value = Math.max(0.0, Math.min(1.0, value));
		if (Math.abs(value - curl) < 1e-4) return;
		curl = value;
		for (finger in fingers) for (segment in finger) {
			var turn = segment.angle * value;
			if (turn < 1e-4) instance.clearJointRotation(segment.joint);
			else instance.setJointRotation(segment.joint, [-Math.sin(turn * 0.5), 0.0, 0.0, Math.cos(turn * 0.5)]);
		}
	}
}

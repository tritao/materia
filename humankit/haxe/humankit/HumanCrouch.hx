package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;

/**
 * A one-way clip of going down into a crouch, measured so that any depth between standing and crouched can be posed
 * from it: how far the pelvis has dropped at each moment of the clip, as a fraction of its whole drop. Holding the
 * clip at the time where the drop is a given fraction gives an authored pose at that depth, in place of a blend of
 * standing and crouching poses.
 */
class HumanCrouch {
	static inline var SAMPLES:Int = 64;
	/** The least the pelvis must drop, in metres, for the clip to count as a crouch. */
	static inline var LEAST_DROP:Float = 0.15;

	public final clip:Int;
	/** Seconds into the clip at which the drop first reaches k / (SAMPLES - 1) of the whole, never going back. */
	final times:Array<Float>;

	function new(clip:Int, times:Array<Float>) {
		this.clip = clip;
		this.times = times;
	}

	/** The time in the clip at which the body is `depth` of the way down (0 standing, 1 crouched). */
	public function timeFor(depth:Float):Float {
		var position = Math.max(0.0, Math.min(1.0, depth)) * (times.length - 1);
		var low = Std.int(Math.floor(position)), high = Std.int(Math.min(times.length - 1, low + 1));
		return times[low] + (times[high] - times[low]) * (position - low);
	}

	/**
	 * Measures a clip of going down. With `toLowest` the clip is taken only as far as the pelvis gets lowest, which is
	 * for a clip that goes down, does something, and comes back up: its descent is the going-down part.
	 */
	public static function measure(asset:AnimationAsset, rig:HumanoidRig, clip:Int, toLowest:Bool = false):Null<HumanCrouch> {
		if (clip < 0 || clip >= asset.clipNames.length) return null;
		var duration = asset.clipDurations[clip];
		var instance = new AnimationInstance(asset);
		var pose = new HumanPose(rig, instance.readJointMatrices());
		var heights:Array<Float> = [];
		for (sample in 0...SAMPLES) {
			instance.setLayer(0, clip, duration * sample / (SAMPLES - 1), 1.0, false);
			instance.setLayer(1, -1, 0.0, 0.0);
			instance.evaluate();
			pose.update(instance.readJointMatrices());
			var pelvis = pose.bonePosition(Pelvis);
			if (pelvis == null) {
				instance.dispose();
				return null;
			}
			heights.push(pelvis[2]);
		}
		instance.dispose();
		var last = SAMPLES - 1;
		if (toLowest) for (sample in 0...SAMPLES) if (heights[sample] < heights[last]) last = sample;
		var drop = heights[0] - heights[last];
		if (drop < LEAST_DROP) return null;
		// For each depth, the first moment the pelvis has dropped that far, so a clip that dips and rises still has a monotone map.
		var times:Array<Float> = [];
		for (level in 0...SAMPLES) {
			var depth = level / (SAMPLES - 1);
			var found = last;
			for (sample in 0...last + 1)
				if ((heights[0] - heights[sample]) / drop >= depth - 1e-9) { found = sample; break; }
			times.push(duration * found / (SAMPLES - 1));
		}
		return new HumanCrouch(clip, times);
	}
}

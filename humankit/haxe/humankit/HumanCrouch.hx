package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import humankit.rig.HumanPose;
import humankit.rig.HumanoidRig;

/**
 * A one-way clip of going down into a crouch, measured so that any depth between standing and crouched can be posed
 * from it: how far the pelvis has dropped at each moment of the clip, as a fraction of its whole drop. Holding the
 * clip at the time where the drop is a given fraction gives an authored pose at that depth, in place of a blend of
 * standing and crouching poses.
 */
class HumanCrouch {
	/** How many moments of the clip are measured; between them the moment of a given drop is interpolated. */
	static inline var SAMPLES:Int = 128;
	/** How many depths the map holds; between them the time is interpolated. */
	static inline var LEVELS:Int = 64;
	/** The least the pelvis must drop, in metres, for the clip to count as a crouch. */
	static inline var LEAST_DROP:Float = 0.15;

	public final clip:Int;
	/** Seconds into the clip at which the drop first reaches k / (LEVELS - 1) of the whole, never going back. */
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
		// The moment is found between two samples by the drop's own line, not snapped to the nearer one: a map that snaps is a
		// staircase, and a body held at its steps jumps by a whole sample of the clip wherever the clip moves fast.
		var times:Array<Float> = [];
		for (level in 0...LEVELS) {
			var depth = level / (LEVELS - 1);
			var moment = last * 1.0;
			for (sample in 0...last + 1) {
				var dropped = (heights[0] - heights[sample]) / drop;
				if (dropped >= depth - 1e-9) {
					moment = sample * 1.0;
					if (sample > 0) {
						var before = (heights[0] - heights[sample - 1]) / drop;
						if (dropped > before + 1e-12) moment = sample - 1 + Math.max(0.0, Math.min(1.0, (depth - before) / (dropped - before)));
					}
					break;
				}
			}
			times.push(duration * moment / (SAMPLES - 1));
		}
		return new HumanCrouch(clip, times);
	}
}

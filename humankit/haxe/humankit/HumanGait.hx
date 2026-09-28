package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;

/**
 * The natural ground speed of a looping locomotion clip. While a foot is
 * planted it slides backwards past the pelvis at exactly the speed the body
 * travels, so playing the clip at `speed / naturalSpeed` keeps planted feet
 * still on the floor. Clips are expected in place, facing +X.
 */
class HumanGait {
	static inline var SAMPLES:Int = 64;
	/** A foot within this height of its lowest point counts as planted, in metres. */
	static inline var PLANTED_HEIGHT:Float = 0.015;

	public final clip:Int;
	/** Metres per second at playback rate 1. */
	public final naturalSpeed:Float;

	function new(clip:Int, naturalSpeed:Float) {
		this.clip = clip;
		this.naturalSpeed = naturalSpeed;
	}

	/** Measures a clip of asset on rig by sampling one cycle. */
	public static function measure(asset:AnimationAsset, rig:HumanoidRig, clip:Int):HumanGait {
		if (clip < 0 || clip >= asset.clipNames.length)
			throw 'Clip index $clip is out of range';
		var duration = asset.clipDurations[clip];
		if (!(duration > 0.0))
			throw 'Clip "${asset.clipNames[clip]}" has no duration';
		var instance = new AnimationInstance(asset);
		var pose = new HumanPose(rig, instance.readJointMatrices());
		var feet:Array<Array<Array<Float>>> = [[], []];
		for (sample in 0...SAMPLES) {
			instance.setLayer(0, clip, duration * sample / SAMPLES, 1.0, true);
			instance.setLayer(1, -1, 0.0, 0.0);
			instance.evaluate();
			pose.update(instance.readJointMatrices());
			var pelvis = pose.bonePosition(Pelvis);
			var side = 0;
			for (foot in [FootL, FootR]) {
				var position = pose.bonePosition(foot);
				feet[side].push([position[0] - pelvis[0], position[2]]);
				side++;
			}
		}
		instance.dispose();
		var step = duration / SAMPLES;
		var speeds:Array<Float> = [];
		for (track in feet) {
			var lowest = Math.POSITIVE_INFINITY;
			for (sample in track)
				lowest = Math.min(lowest, sample[1]);
			for (index in 0...SAMPLES) {
				if (track[index][1] > lowest + PLANTED_HEIGHT)
					continue;
				var before = track[(index + SAMPLES - 1) % SAMPLES], after = track[(index + 1) % SAMPLES];
				// Backwards past the pelvis is forwards for the body.
				speeds.push(-(after[0] - before[0]) / (2.0 * step));
			}
		}
		if (speeds.length == 0)
			throw 'Clip "${asset.clipNames[clip]}" never plants a foot';
		speeds.sort(Reflect.compare);
		var median = speeds[Std.int(speeds.length / 2)];
		if (!(median > 0.0))
			throw 'Clip "${asset.clipNames[clip]}" does not move forwards';
		return new HumanGait(clip, median);
	}
}

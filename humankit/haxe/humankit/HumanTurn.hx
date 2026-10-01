package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;

/**
 * A turn-in-place clip, measured: how far the body has turned by the end of the clip and how long that takes.
 * Such a clip turns the body in its own space and does not move the root, so it ends in the pose of the idle
 * clip turned by that angle. Playing it and then turning the root by the same angle while switching to idle is
 * seamless, and the feet step through the turn instead of sliding. The turn is read from the line between the
 * shoulders, which a clip that turns the body turns with it.
 */
class HumanTurn {
	static inline var SAMPLES:Int = 48;
	/** The angle from which the body counts as done turning, in radians (about a degree and a half). */
	static inline var SETTLED:Float = 0.026;

	public final clip:Int;
	/** Radians turned, positive to the left (counter-clockwise from above). */
	public final angle:Float;
	/** Seconds of the clip until the turn is done; the rest of it only holds the pose. */
	public final seconds:Float;

	function new(clip:Int, angle:Float, seconds:Float) {
		this.clip = clip;
		this.angle = angle;
		this.seconds = seconds;
	}

	/** Measures a clip, or null when it does not turn the body by roughly a right angle. */
	public static function measure(asset:AnimationAsset, rig:HumanoidRig, clip:Int):Null<HumanTurn> {
		if (clip < 0 || clip >= asset.clipNames.length) return null;
		var duration = asset.clipDurations[clip];
		var instance = new AnimationInstance(asset);
		var pose = new HumanPose(rig, instance.readJointMatrices());
		var yaws:Array<Float> = [];
		var usable = true;
		for (sample in 0...SAMPLES) {
			instance.setLayer(0, clip, duration * sample / SAMPLES, 1.0, false);
			instance.setLayer(1, -1, 0.0, 0.0);
			instance.evaluate();
			pose.update(instance.readJointMatrices());
			var left = pose.bonePosition(UpperArmL), right = pose.bonePosition(UpperArmR);
			if (left == null || right == null) { usable = false; break; }
			yaws.push(Math.atan2(left[1] - right[1], left[0] - right[0]));
		}
		instance.dispose();
		if (!usable) return null;
		// Unwrapped, relative to the first sample.
		var turned:Array<Float> = [];
		var previous = 0.0;
		for (yaw in yaws) {
			var step = yaw - yaws[0] - previous;
			while (step > Math.PI) step -= 2.0 * Math.PI;
			while (step < -Math.PI) step += 2.0 * Math.PI;
			previous += step;
			turned.push(previous);
		}
		var total = turned[Std.int(SAMPLES * 0.95)];
		if (Math.abs(total) < 1.0 || Math.abs(total) > 2.1) return null;
		var done = SAMPLES - 1;
		for (sample in 0...SAMPLES)
			if (Math.abs(turned[sample] - total) < SETTLED) { done = sample; break; }
		return new HumanTurn(clip, total, duration * done / SAMPLES);
	}
}

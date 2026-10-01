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
	/** How long the idle pose takes to fade into a walk, as `HumanWalker` fades it. */
	static inline var FADE:Float = 0.3;

	public final clip:Int;
	/** Metres per second at playback rate 1. */
	public final naturalSpeed:Float;
	/**
	 * Seconds into the clip where a walk from standing should begin: the moment in the cycle when the feet are nearest
	 * where they stand at idle, one foot planted under the body and the other passing it. Starting there, the fade from idle
	 * drags the feet a few centimetres; starting at the top of the cycle drags them across a stride.
	 */
	public final startTime:Float;

	function new(clip:Int, naturalSpeed:Float, startTime:Float) {
		this.clip = clip;
		this.naturalSpeed = naturalSpeed;
		this.startTime = startTime;
	}

	/**
	 * How far a foot that stands at idle would slide, predicted, if a walk began at `candidate` (a sample of the cycle):
	 * the idle pose fades into the clip over `FADE` seconds while the body accelerates to `speed`, the clip running at the
	 * body's pace. A foot counts as planted while it is low and slow, as `Naturalness` counts it, and its slide is how far it
	 * has moved from where it stood. The feet are blended in the pelvis's frame, which is close to how the poses blend.
	 */
	static function startSlide(across:Array<Array<Array<Float>>>, feet:Array<Array<Array<Float>>>, standing:Array<Array<Float>>,
			duration:Float, speed:Float, candidate:Int, holdsFeet:Bool):Float {
		var posture = new HumanPosture();
		var step = 0.01, total = 1.2;
		var time = duration * candidate / SAMPLES, travelled = 0.0;
		var plantedAt:Array<Null<Array<Float>>> = [[standing[0][0], standing[0][1]], [standing[1][0], standing[1][1]]];
		var lowFor = [0, 0];
		var last:Array<Array<Float>> = [[standing[0][0], standing[0][1]], [standing[1][0], standing[1][1]]];
		var worst = 0.0;
		var count = Std.int(total / step);
		for (index in 1...count + 1) {
			var seconds = index * step;
			var velocity = Math.min(speed, speed / FADE * seconds);
			travelled += velocity * step;
			time += velocity / speed * step;
			var weight = Math.min(1.0, seconds / FADE);
			var position = (time % duration) / duration * SAMPLES;
			var low = Std.int(Math.floor(position)) % SAMPLES, high = (low + 1) % SAMPLES, share = position - Math.floor(position);
			for (side in 0...2) {
				var x = across[side][low][0] + (across[side][high][0] - across[side][low][0]) * share;
				var y = across[side][low][1] + (across[side][high][1] - across[side][low][1]) * share;
				var z = feet[side][low][1] + (feet[side][high][1] - feet[side][low][1]) * share;
				var worldX = travelled + (1.0 - weight) * standing[side][0] + weight * x;
				var worldY = (1.0 - weight) * standing[side][1] + weight * y;
				if (holdsFeet) {
					// A body that holds its feet pins them where they stood while the idle pose still shows (see `HumanBody.holdFeet`).
					var wanted = Math.max(0.0, Math.min(1.0, ((1.0 - weight) - posture.lockFrom) / posture.lockSpan));
					wanted = wanted * wanted * (3.0 - 2.0 * wanted);
					worldX += (standing[side][0] - worldX) * wanted;
					worldY += (standing[side][1] - worldY) * wanted;
				}
				var height = (1.0 - weight) * standing[side][2] + weight * z;
				var moved = Math.sqrt(Math.pow(worldX - last[side][0], 2) + Math.pow(worldY - last[side][1], 2)) / step;
				last[side] = [worldX, worldY];
				var onFloor = height <= standing[side][2] + Naturalness.PLANTED_HEIGHT;
				var origin = plantedAt[side];
				if (origin == null) {
					lowFor[side] = onFloor && moved < Naturalness.PLANTED_SPEED ? lowFor[side] + 1 : 0;
					if (lowFor[side] >= 3) plantedAt[side] = [worldX, worldY];
				} else if (!onFloor || moved > Naturalness.UNPLANTED_SPEED) {
					plantedAt[side] = null;
					lowFor[side] = 0;
				} else
					worst = Math.max(worst, Math.sqrt(Math.pow(worldX - origin[0], 2) + Math.pow(worldY - origin[1], 2)));
			}
		}
		return worst;
	}

	/**
	 * Measures a clip of asset on rig by sampling one cycle. With `idleClip`, also finds the moment in the cycle
	 * that best matches the feet at idle (see `startTime`); without it a walk starts at the top of the cycle.
	 */
	public static function measure(asset:AnimationAsset, rig:HumanoidRig, clip:Int, backward:Bool = false, idleClip:Int = -1, holdsFeet:Bool = false):HumanGait {
		if (clip < 0 || clip >= asset.clipNames.length)
			throw 'Clip index $clip is out of range';
		var duration = asset.clipDurations[clip];
		if (!(duration > 0.0))
			throw 'Clip "${asset.clipNames[clip]}" has no duration';
		var instance = new AnimationInstance(asset);
		var pose = new HumanPose(rig, instance.readJointMatrices());
		var feet:Array<Array<Array<Float>>> = [[], []];
		// Each foot's place against the pelvis across the floor, for finding the moment that matches the idle stance.
		var across:Array<Array<Array<Float>>> = [[], []];
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
				across[side].push([position[0] - pelvis[0], position[1] - pelvis[1]]);
				side++;
			}
		}
		var standing:Null<Array<Array<Float>>> = null;
		if (idleClip >= 0 && idleClip < asset.clipNames.length) {
			instance.setLayer(0, idleClip, 0.0, 1.0, true);
			instance.setLayer(1, -1, 0.0, 0.0);
			instance.evaluate();
			pose.update(instance.readJointMatrices());
			var pelvis = pose.bonePosition(Pelvis);
			standing = [for (foot in [FootL, FootR]) {
				var position = pose.bonePosition(foot);
				[position[0] - pelvis[0], position[1] - pelvis[1], position[2]];
			}];
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
				// (A backward gait is the other way round: the planted foot slides forwards past the pelvis.)
				speeds.push((backward ? 1.0 : -1.0) * (after[0] - before[0]) / (2.0 * step));
			}
		}
		if (speeds.length == 0)
			throw 'Clip "${asset.clipNames[clip]}" never plants a foot';
		speeds.sort(Reflect.compare);
		var median = speeds[Std.int(speeds.length / 2)];
		if (!(median > 0.0))
			throw 'Clip "${asset.clipNames[clip]}" does not move ${backward ? "backwards" : "forwards"}';
		var start = 0.0;
		if (standing != null) {
			// Try every phase of the cycle as the start, and take the one whose fade from idle drags a foot least.
			var best = Math.POSITIVE_INFINITY;
			for (candidate in 0...SAMPLES) {
				var slide = startSlide(across, feet, standing, duration, median, candidate, holdsFeet);
				if (slide < best - 1e-6) {
					best = slide;
					start = duration * candidate / SAMPLES;
				}
			}
		}
		return new HumanGait(clip, median, start);
	}
}

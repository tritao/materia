package humankit.quality;

import humankit.HumanBody;
import humankit.rig.HumanBone;

/**
 * What a run of poses looks like from the floor up, sampled once per tick: whether planted feet slide, whether the
 * centre of mass stays over the feet while both are down, and how jerky the pelvis and wrists are.
 * `MotionQuality` judges the arms; this judges the body. Neither says a motion is natural, only that a regression
 * showed up as a number.
 *
 * A foot is planted from when it is low (within `PLANTED_HEIGHT` of where it first stood) and slow (under
 * `PLANTED_SPEED`) for `PLANTED_SAMPLES` samples running, until it lifts or speeds up (a swing foot skimming the
 * floor is fast, so is not taken for planted). Its slide is how far it moves from where it was planted while it stays
 * so. The centre of mass is a weighted mean of a few bones, and the support polygon is the hull of both feet from
 * heel to toe; the margin is how far inside it the mass projects (negative outside), kept only while both feet are planted.
 */
class Naturalness {
	static inline var FOOT_LENGTH = 0.15;
	static inline var HEEL_BEHIND = 0.07;
	static inline var FOOT_HALF_WIDTH = 0.04;
	/** Samples between those the jerk is taken from: a third difference of every tick is mostly noise. */
	static inline var JERK_STRIDE = 4;

	public static inline var PLANTED_HEIGHT = 0.02;
	/** Slower than this a low foot is planted; faster than `UNPLANTED_SPEED` it has left the floor. */
	public static inline var PLANTED_SPEED = 0.4;
	public static inline var UNPLANTED_SPEED = 0.9;
	static inline var PLANTED_SAMPLES = 3;

	/** The furthest a foot moved from where it was planted while it stayed planted, in metres. */
	public var maxSlide(default, null):Float = 0.0;
	/** The sample at which that furthest slide was reached, and the one at which that foot was planted. */
	public var maxSlideAt(default, null):Int = -1;
	public var maxSlideFrom(default, null):Int = -1;
	/** The slide of every plant added up, in metres, and the seconds a foot was planted. */
	public var slideTotal(default, null):Float = 0.0;
	public var plantedSeconds(default, null):Float = 0.0;
	/** Least margin of the centre of mass inside the feet's support, in metres, while both were planted; infinite if never. */
	public var minSupportMargin(default, null):Float = Math.POSITIVE_INFINITY;
	/** Lowest a foot went below where it first stood, in metres (positive: it went into the floor). */
	public var floorPenetration(default, null):Float = 0.0;
	/** Largest jerk of the pelvis and of either wrist, in metres per second cubed. */
	public var maxPelvisJerk(default, null):Float = 0.0;
	public var maxWristJerk(default, null):Float = 0.0;
	/** The most a planted foot tilted from the way it first stood (ankle to toe, pitched up or down), in degrees. */
	public var maxFootTilt(default, null):Float = 0.0;
	/** Largest jerk of either ankle: a foot held in the world and let go again must not pop. */
	public var maxFootJerk(default, null):Float = 0.0;
	public var samples(default, null):Int = 0;

	var restHeight:Array<Float> = [];
	var restPitch:Array<Null<Float>> = [null, null];
	var lastFoot:Array<Null<Array<Float>>> = [null, null];
	var lowFor:Array<Int> = [0, 0];
	var plantedAt:Array<Null<Array<Float>>> = [null, null];
	var plantedSample:Array<Int> = [0, 0];
	var slideNow:Array<Float> = [0.0, 0.0];
	final history:Array<Array<Array<Float>>> = [[], [], [], [], []];
	var tick:Int = 0;

	public function new() {}

	/** Takes one sample of a body whose pose is current; `seconds` is the time since the last. */
	/**
	 * Takes the floor under each foot from another run's: a foot's floor height is taken from its first sample, which is only
	 * right when the run begins with both feet down. A run that begins mid-stride says so by taking it from one that stood.
	 */
	public function standOnTheFloorOf(other:Naturalness):Void
		restHeight = other.restHeight.copy();

	public function sample(body:HumanBody, seconds:Float):Void {
		samples++;
		var pose = body.character.pose;
		var feet = [HumanBone.FootL, HumanBone.FootR], toes = [HumanBone.ToeL, HumanBone.ToeR];
		var forward = body.rootTransform();
		var points:Array<Array<Float>> = [];
		var planted = [false, false];
		for (side in 0...2) {
			var local = pose.bonePosition(feet[side]);
			if (local == null) continue;
			var world = body.toWorld(local);
			if (restHeight.length < 2) restHeight.push(world[2]);
			if (restHeight.length <= side) continue;
			floorPenetration = Math.max(floorPenetration, restHeight[side] - world[2]);
			var previous = lastFoot[side];
			var speed = previous == null || seconds <= 0.0 ? 0.0 :
				Math.sqrt(Math.pow(world[0] - previous[0], 2) + Math.pow(world[1] - previous[1], 2)) / seconds;
			var low = world[2] <= restHeight[side] + PLANTED_HEIGHT;
			if (plantedAt[side] == null) {
				lowFor[side] = low && speed < PLANTED_SPEED ? lowFor[side] + 1 : 0;
				if (lowFor[side] >= PLANTED_SAMPLES) {
					plantedAt[side] = world;
					plantedSample[side] = samples;
					slideNow[side] = 0.0;
				}
			} else if (!low || speed > UNPLANTED_SPEED) {
				slideTotal += slideNow[side];
				plantedAt[side] = null;
				lowFor[side] = 0;
			} else {
				var origin = plantedAt[side];
				slideNow[side] = Math.sqrt(Math.pow(world[0] - origin[0], 2) + Math.pow(world[1] - origin[1], 2));
				if (slideNow[side] > maxSlide) {
					maxSlide = slideNow[side];
					maxSlideAt = samples;
					maxSlideFrom = plantedSample[side];
				}
			}
			planted[side] = plantedAt[side] != null;
			var toeAt = pose.bonePosition(toes[side]);
			if (toeAt != null) {
				var run = Math.sqrt(Math.pow(toeAt[0] - local[0], 2) + Math.pow(toeAt[1] - local[1], 2));
				var pitch = Math.atan2(toeAt[2] - local[2], run) * 180.0 / Math.PI;
				var rest = restPitch[side];
				if (rest == null) restPitch[side] = pitch;
				else if (planted[side]) maxFootTilt = Math.max(maxFootTilt, Math.abs(pitch - rest));
			}
			if (planted[side]) plantedSeconds += seconds;
			lastFoot[side] = world;
			// The foot's outline: from a heel behind the ankle to the toe (its bone when the rig has one, else a foot's
			// length ahead), a hand's width across.
			var toe = pose.bonePosition(toes[side]);
			var tipX = toe == null ? local[0] + FOOT_LENGTH : toe[0];
			for (x in [local[0] - HEEL_BEHIND, tipX]) for (y in [local[1] - FOOT_HALF_WIDTH, local[1] + FOOT_HALF_WIDTH]) {
				var corner = body.toWorld([x, y, local[2]]);
				points.push([corner[0], corner[1]]);
			}
		}
		if (planted[0] && planted[1] && points.length == 8) {
			var mass = centreOfMass(body);
			if (mass != null) minSupportMargin = Math.min(minSupportMargin, marginInside(hull(points), mass[0], mass[1]));
		}
		tick++;
		if (tick % JERK_STRIDE != 0) return;
		var tracked = [HumanBone.Pelvis, HumanBone.HandL, HumanBone.HandR, HumanBone.FootL, HumanBone.FootR];
		for (index in 0...5) {
			var local = pose.bonePosition(tracked[index]);
			if (local == null) continue;
			var series = history[index];
			series.push(body.toWorld(local));
			if (series.length > 4) series.shift();
			if (series.length < 4) continue;
			var step = seconds * JERK_STRIDE;
			var k = 0.0;
			for (axis in 0...3)
				k += Math.pow((series[3][axis] - 3.0 * series[2][axis] + 3.0 * series[1][axis] - series[0][axis]) / (step * step * step), 2);
			k = Math.sqrt(k);
			if (index == 0) maxPelvisJerk = Math.max(maxPelvisJerk, k);
			else if (index < 3) maxWristJerk = Math.max(maxWristJerk, k);
			else maxFootJerk = Math.max(maxFootJerk, k);
		}
	}

	/** A rough centre of mass in the world: the pelvis, trunk, head, and knees, weighted. Null if the rig lacks them. */
	static function centreOfMass(body:HumanBody):Null<Array<Float>> {
		var bones = [HumanBone.Pelvis, HumanBone.Spine, HumanBone.Chest, HumanBone.Head, HumanBone.ShinL, HumanBone.ShinR];
		var weights = [0.30, 0.15, 0.30, 0.08, 0.085, 0.085];
		var total = [0.0, 0.0, 0.0];
		for (index in 0...bones.length) {
			var local = body.character.pose.bonePosition(bones[index]);
			if (local == null) return null;
			var world = body.toWorld(local);
			for (axis in 0...3) total[axis] += world[axis] * weights[index];
		}
		return total;
	}

	/** The convex hull of 2D points, counter-clockwise (Andrew's monotone chain). */
	static function hull(points:Array<Array<Float>>):Array<Array<Float>> {
		var sorted = points.copy();
		sorted.sort((a, b) -> a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : a[1] < b[1] ? -1 : a[1] > b[1] ? 1 : 0);
		var cross = function(o:Array<Float>, a:Array<Float>, b:Array<Float>):Float
			return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
		var lower:Array<Array<Float>> = [];
		for (point in sorted) {
			while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0.0) lower.pop();
			lower.push(point);
		}
		var upper:Array<Array<Float>> = [];
		var index = sorted.length - 1;
		while (index >= 0) {
			var point = sorted[index];
			while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0.0) upper.pop();
			upper.push(point);
			index--;
		}
		lower.pop();
		upper.pop();
		return lower.concat(upper);
	}

	/** How far inside a counter-clockwise polygon a point is, to its nearest edge; negative outside. */
	static function marginInside(polygon:Array<Array<Float>>, x:Float, y:Float):Float {
		var least = Math.POSITIVE_INFINITY;
		for (index in 0...polygon.length) {
			var a = polygon[index], b = polygon[(index + 1) % polygon.length];
			var length = Math.sqrt(Math.pow(b[0] - a[0], 2) + Math.pow(b[1] - a[1], 2));
			if (length < 1e-9) continue;
			least = Math.min(least, ((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0])) / length);
		}
		return least;
	}

	/** One line for a report. */
	public function summary():String {
		var r = function(value:Float, scale:Float):Float return Math.round(value * scale) / scale;
		return 'slide ${r(maxSlide, 100)} m at most (${r(slideTotal, 100)} m in all over ${r(plantedSeconds, 10)} s planted), support margin '
			+ (minSupportMargin == Math.POSITIVE_INFINITY ? "n/a" : '${r(minSupportMargin, 100)} m') + ', floor ${r(floorPenetration, 100)} m, foot tilt ${Math.round(maxFootTilt)} deg, jerk pelvis '
			+ '${Math.round(maxPelvisJerk)} wrist ${Math.round(maxWristJerk)} foot ${Math.round(maxFootJerk)} m/s3';
	}
}

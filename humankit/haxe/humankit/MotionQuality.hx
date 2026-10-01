package humankit;

/** What a run of poses did to one arm, in model space (+Z up, +X forward). */
typedef ArmQuality = {
	/** Fastest turn of the elbow's bend plane, in radians per second; a flip shows as a spike. */
	var planeTurnRate:Float;
	/** Highest the elbow rose above the shoulder, in metres (negative while it stays below). */
	var elbowAboveShoulder:Float;
	/** Sharpest elbow bend, as the angle between upper arm and forearm in degrees (180 is straight). */
	var minElbowAngle:Float;
	/** Fastest wrist, in metres per second. */
	var maxHandSpeed:Float;
	/** Hardest wrist acceleration, in metres per second squared. */
	var maxHandAcceleration:Float;
	/** Closest the wrist came to the front of the chest, in metres; negative is behind it. */
	var handAheadOfChest:Float;
	/** The sample index at which the plane turn, hand speed and hand acceleration peaked. */
	var planeTurnAt:Int;
	var handSpeedAt:Int;
	var handAccelerationAt:Int;
	/** The sample index at which the elbow bent sharpest. */
	var minElbowAt:Int;
}

/**
 * Measures how natural a motion is from its poses, one sample per tick: arms that flip, fold up to
 * the shoulder, bend too sharply, whip, or are dragged behind the torso. A motion's tests can
 * hold these to limits, and a regression then shows up as a number.
 */
class MotionQuality {
	public static inline var LEFT = 0;
	public static inline var RIGHT = 1;

	/** A bend plane is only defined while the elbow is bent: straighter than this has none. */
	static inline var STRAIGHT_DEGREES = 165.0;

	final arms:Array<ArmQuality> = [for (_ in 0...2) MotionQuality.fresh()];
	final normals:Array<Null<Array<Float>>> = [null, null];
	final hands:Array<Null<Array<Float>>> = [null, null];
	final velocities:Array<Null<Array<Float>>> = [null, null];
	public var samples(default, null):Int = 0;

	public function new() {}

	static function fresh():ArmQuality
		return {planeTurnRate: 0.0, elbowAboveShoulder: Math.NEGATIVE_INFINITY, minElbowAngle: 180.0,
			maxHandSpeed: 0.0, maxHandAcceleration: 0.0, handAheadOfChest: Math.POSITIVE_INFINITY,
			planeTurnAt: -1, handSpeedAt: -1, handAccelerationAt: -1, minElbowAt: -1};

	public function arm(side:Int):ArmQuality
		return arms[side];

	/** Adds the pose reached `seconds` after the previous sample (any value the first time). */
	public function sample(pose:HumanPose, seconds:Float):Void {
		samples++;
		var chest = pose.bonePosition(Chest);
		if (chest == null) chest = pose.bonePosition(Pelvis);
		for (side in 0...2) {
			var left = side == LEFT;
			var shoulder = pose.bonePosition(left ? UpperArmL : UpperArmR);
			var elbow = pose.bonePosition(left ? ForearmL : ForearmR);
			var hand = pose.bonePosition(left ? HandL : HandR);
			if (shoulder == null || elbow == null || hand == null) continue;
			var result = arms[side];
			var upper = Mat4.subtract(shoulder, elbow), lower = Mat4.subtract(hand, elbow);
			var cosine = Mat4.dot(upper, lower) / Math.max(1e-9, length(upper) * length(lower));
			var angle = Math.acos(Math.max(-1.0, Math.min(1.0, cosine))) * 180.0 / Math.PI;
			if (angle < result.minElbowAngle) {
				result.minElbowAngle = angle;
				result.minElbowAt = samples;
			}
			result.elbowAboveShoulder = Math.max(result.elbowAboveShoulder, elbow[2] - shoulder[2]);
			if (chest != null) result.handAheadOfChest = Math.min(result.handAheadOfChest, hand[0] - chest[0]);

			var normal:Null<Array<Float>> = null;
			if (angle < STRAIGHT_DEGREES) {
				var across = Mat4.cross(Mat4.subtract(elbow, shoulder), lower);
				if (length(across) > 1e-9) normal = Mat4.normalize(across);
			}
			var before = normals[side];
			if (normal != null && before != null && seconds > 0.0) {
				var turn = Math.acos(Math.max(-1.0, Math.min(1.0, Mat4.dot(normal, before))));
				if (turn / seconds > result.planeTurnRate) {
					result.planeTurnRate = turn / seconds;
					result.planeTurnAt = samples;
				}
			}
			normals[side] = normal;

			var last = hands[side];
			if (last != null && seconds > 0.0) {
				var velocity = [for (axis in 0...3) (hand[axis] - last[axis]) / seconds];
				if (length(velocity) > result.maxHandSpeed) {
					result.maxHandSpeed = length(velocity);
					result.handSpeedAt = samples;
				}
				var previous = velocities[side];
				if (previous != null) {
					var acceleration = length([for (axis in 0...3) (velocity[axis] - previous[axis]) / seconds]);
					if (acceleration > result.maxHandAcceleration) {
						result.maxHandAcceleration = acceleration;
						result.handAccelerationAt = samples;
					}
				}
				velocities[side] = velocity;
			}
			hands[side] = hand;
		}
	}

	/** Forgets the previous sample, so a jump the caller caused (a reset) is not counted as motion. */
	public function breakContinuity():Void {
		for (side in 0...2) {
			normals[side] = null;
			hands[side] = null;
			velocities[side] = null;
		}
	}

	static function length(v:Array<Float>):Float
		return Math.sqrt(Mat4.dot(v, v));
}

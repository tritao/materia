package humankit;

/**
 * A person's body measurements in metres. Measured from a rig's rest pose by
 * default; scaledTo() describes the same body at another stature, and
 * `scale` is then the uniform scale to apply to the character's visual root.
 * Side-dependent lengths are the mean of both sides.
 */
class HumanDescription {
	/** Floor to crown. */
	public final stature:Float;
	/** Between the upper arm joints. */
	public final shoulderWidth:Float;
	/** Between the thigh joints. */
	public final hipWidth:Float;
	/** Pelvis joint to neck joint. */
	public final torso:Float;
	public final upperArm:Float;
	public final forearm:Float;
	public final thigh:Float;
	public final shin:Float;
	/** Uniform scale from the measured rig to this description. */
	public final scale:Float;

	public function new(stature:Float, shoulderWidth:Float, hipWidth:Float, torso:Float, upperArm:Float,
			forearm:Float, thigh:Float, shin:Float, scale:Float = 1.0) {
		if (!(stature > 0.0) || !(scale > 0.0))
			throw "A human description needs a positive stature and scale";
		this.stature = stature;
		this.shoulderWidth = shoulderWidth;
		this.hipWidth = hipWidth;
		this.torso = torso;
		this.upperArm = upperArm;
		this.forearm = forearm;
		this.thigh = thigh;
		this.shin = shin;
		this.scale = scale;
	}

	/** Measures a rest pose; stature is the model's rest height (floor to crown). */
	public static function measure(rest:HumanPose, stature:Float):HumanDescription {
		return new HumanDescription(stature, distance(rest, UpperArmL, UpperArmR), distance(rest, ThighL, ThighR),
			distance(rest, Pelvis, Neck), bilateral(rest, UpperArmL, ForearmL, UpperArmR, ForearmR),
			bilateral(rest, ForearmL, HandL, ForearmR, HandR), bilateral(rest, ThighL, ShinL, ThighR, ShinR),
			bilateral(rest, ShinL, FootL, ShinR, FootR));
	}

	/** The same body proportions at another stature. */
	public function scaledTo(targetStature:Float):HumanDescription {
		if (!(targetStature > 0.0))
			throw "A human stature must be positive";
		var k = targetStature / stature;
		return new HumanDescription(targetStature, shoulderWidth * k, hipWidth * k, torso * k, upperArm * k, forearm * k,
			thigh * k, shin * k, scale * k);
	}

	static function bilateral(pose:HumanPose, leftFrom:HumanBone, leftTo:HumanBone, rightFrom:HumanBone,
			rightTo:HumanBone):Float
		return (distance(pose, leftFrom, leftTo) + distance(pose, rightFrom, rightTo)) * 0.5;

	static function distance(pose:HumanPose, a:HumanBone, b:HumanBone):Float {
		var from = pose.bonePosition(a), to = pose.bonePosition(b);
		if (from == null || to == null)
			throw 'Cannot measure $a to $b: the rig lacks one of them';
		var delta = Mat4.subtract(to, from);
		return Math.sqrt(Mat4.dot(delta, delta));
	}
}

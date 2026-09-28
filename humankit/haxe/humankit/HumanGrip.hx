package humankit;

/** Prop offsets for HumanCharacter.attach, expressed in the standard hand frame (see HumanPose). */
class HumanGrip {
	/**
	 * A closed-fist grip on a handle. The prop's +X axis runs through the fist
	 * along the hand's thumb side (+Y), held gripPoint metres along the prop from
	 * its origin, and palmDepth metres from the palm centre towards the fingers.
	 */
	public static function handle(gripPoint:Float, palmDepth:Float = 0.03):Array<Float> {
		var rotation = Mat4.frame([palmDepth, 0.0, 0.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]);
		return Mat4.multiply(rotation, Mat4.translation(-gripPoint, 0.0, 0.0));
	}
}

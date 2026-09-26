package motionkit.trajectory;

/**
 * Scalar limits used by a trajectory planner.
 *
 * A zero value means that the caller has not supplied a usable limit yet; a
 * planner must resolve it from the machine or reject the request. Jerk is
 * carried from the beginning so planners can add S-curve profiles without
 * changing the public motion API.
 */
class MotionLimits {
  public final maxVelocity:Float;
  public final maxAcceleration:Float;
  public final maxJerk:Float;

  public function new(?maxVelocity:Float = 0.0, ?maxAcceleration:Float = 0.0,
      ?maxJerk:Float = 0.0) {
    requireNonnegative(maxVelocity, "maximum velocity");
    requireNonnegative(maxAcceleration, "maximum acceleration");
    requireNonnegative(maxJerk, "maximum jerk");
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxJerk = maxJerk;
  }

  static function requireNonnegative(value:Float, label:String):Void {
    if (!Math.isFinite(value) || value < 0.0)
      throw 'Motion $label must be finite and non-negative';
  }
}

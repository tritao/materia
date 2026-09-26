package motionkit.axis;

/** Per-command limits. Zero means use the compiled machine limit. */
class MotionOptions {
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
      throw 'Motion option $label must be finite and non-negative';
  }
}

package motionkit;

/** Root-package facade for per-command motion limits. */
class MotionOptions extends motionkit.axis.MotionOptions {
  public function new(?maxVelocity:Float = 0.0, ?maxAcceleration:Float = 0.0,
      ?maxJerk:Float = 0.0)
    super(maxVelocity, maxAcceleration, maxJerk);
}

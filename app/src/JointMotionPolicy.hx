package app;

import robotkit.model.JointLimits;
import robotkit.model.JointType;

typedef JointMotionRates = {
  var velocity:Float;
  var acceleration:Float;
  var jerk:Float;
}

/** Requested mission rates, in radians or metres per second powers. These are planning
 * settings, not hardware ratings. Mechanical ceilings still constrain every joint. */
class JointMotionPolicy {
  public final angularAcceleration:Float;
  public final angularJerk:Float;
  public final linearAcceleration:Float;
  public final linearJerk:Float;

  public function new(angularAcceleration:Float, angularJerk:Float,
      linearAcceleration:Float, linearJerk:Float) {
    for (value in [angularAcceleration, angularJerk, linearAcceleration, linearJerk])
      if (!Math.isFinite(value) || value <= 0) throw "Requested joint-motion rates must be finite and positive";
    this.angularAcceleration = angularAcceleration;
    this.angularJerk = angularJerk;
    this.linearAcceleration = linearAcceleration;
    this.linearJerk = linearJerk;
  }

  /** The application's existing mission settings, stated separately for each joint unit. */
  public static function mission():JointMotionPolicy
    return new JointMotionPolicy(2.0, 20.0, 2.0, 20.0);

  public function rates(type:JointType, limits:JointLimits):JointMotionRates {
    if (type != JointType.Revolute && type != JointType.Continuous && type != JointType.Prismatic)
      throw "Joint-motion policy requires a scalar moving joint";
    var error = limits.validate();
    if (error != null) throw 'Invalid mechanical joint limits: $error';
    var velocity = limits.requireVelocity();
    if (!Math.isFinite(velocity) || velocity <= 0)
      throw "Joint motion needs a finite positive mechanical velocity limit";
    var acceleration = type == JointType.Prismatic ? linearAcceleration : angularAcceleration;
    var jerk = type == JointType.Prismatic ? linearJerk : angularJerk;
    var ceiling = limits.maxAcceleration;
    if (ceiling != null) {
      if (ceiling <= 0) throw "The mechanical acceleration limit does not permit joint motion";
      acceleration = Math.min(acceleration, ceiling);
    }
    return {velocity: velocity, acceleration: acceleration, jerk: jerk};
  }
}

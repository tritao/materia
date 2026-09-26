package robotkit.mobile;

/** Configurable speed and acceleration limits for planar base commands. */
class MotionLimits {
  public final maxLinearSpeed:Float;
  public final maxAngularSpeed:Float;
  public final maxLinearAcceleration:Float;
  public final maxAngularAcceleration:Float;
  public final maxLateralSpeed:Float;
  public final maxLateralAcceleration:Float;

  public function new(maxLinearSpeed:Float, maxAngularSpeed:Float,
      ?maxLinearAcceleration:Float = 1.0e300,
      ?maxAngularAcceleration:Float = 1.0e300,
      ?maxLateralSpeed:Float = -1.0,
      ?maxLateralAcceleration:Float = -1.0) {
    requirePositive(maxLinearSpeed, "maxLinearSpeed");
    requirePositive(maxAngularSpeed, "maxAngularSpeed");
    requirePositive(maxLinearAcceleration, "maxLinearAcceleration");
    requirePositive(maxAngularAcceleration, "maxAngularAcceleration");
    var lateralSpeed = maxLateralSpeed < 0.0 ? maxLinearSpeed : maxLateralSpeed;
    var lateralAcceleration = maxLateralAcceleration < 0.0 ? maxLinearAcceleration : maxLateralAcceleration;
    requirePositive(lateralSpeed, "maxLateralSpeed");
    requirePositive(lateralAcceleration, "maxLateralAcceleration");
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularAcceleration = maxAngularAcceleration;
    this.maxLateralSpeed = lateralSpeed;
    this.maxLateralAcceleration = lateralAcceleration;
  }

  /** Applies speed limits, then acceleration limits when a duration is supplied. */
  public function constrain(target:Twist2, previous:Twist2, ?durationSeconds:Float):Twist2 {
    if (target == null) throw "MotionLimits requires a target twist";
    var linear = clamp(target.linear, maxLinearSpeed);
    var angular = clamp(target.angular, maxAngularSpeed);
    var lateral = clamp(target.lateral, maxLateralSpeed);
    if (durationSeconds != null) {
      if (!Math.isFinite(durationSeconds) || durationSeconds <= 0.0)
        throw "Acceleration limiting requires a finite positive duration";
      var origin = previous == null ? Twist2.zero() : previous;
      linear = approach(origin.linear, linear, maxLinearAcceleration * durationSeconds);
      angular = approach(origin.angular, angular, maxAngularAcceleration * durationSeconds);
      lateral = approach(origin.lateral, lateral, maxLateralAcceleration * durationSeconds);
    }
    return new Twist2(linear, angular, lateral);
  }

  static function clamp(value:Float, magnitude:Float):Float {
    return value > magnitude ? magnitude : (value < -magnitude ? -magnitude : value);
  }

  static function approach(from:Float, to:Float, amount:Float):Float {
    if (to > from) return Math.min(to, from + amount);
    return Math.max(to, from - amount);
  }

  static function requirePositive(value:Float, name:String):Void {
    if (Math.isNaN(value) || value <= 0.0)
      throw '$name must be positive';
  }
}

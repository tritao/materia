package robotkit.model;

class JointLimits {
  public var lower:Float;
  public var upper:Float;
  public var velocity:Float;
  public var effort:Float;
  public var maxAcceleration:Float;
  /**
   * How far the joint can travel past `lower` and `upper` before it meets its
   * end stop, in joint units, as a machine's limit switch sits beyond its soft
   * limit. Commands stay within the limits; the runtime faults only past this.
   */
  public var overtravel:Float = 0.0;

  public function new(?lower:Float = 0.0, ?upper:Float = 0.0,
      ?velocity:Float = 0.0, ?effort:Float = 0.0,
      ?maxAcceleration:Float = 0.0) {
    this.lower = lower;
    this.upper = upper;
    this.velocity = velocity;
    this.effort = effort;
    this.maxAcceleration = maxAcceleration;
  }

  public function validate():Null<String> {
    if (lower > upper) return "lower limit exceeds upper limit";
    if (velocity < 0.0) return "velocity limit must be non-negative";
    if (effort < 0.0) return "effort limit must be non-negative";
    if (!Math.isFinite(maxAcceleration) || maxAcceleration < 0.0)
      return "maximum acceleration must be finite and non-negative";
    if (!Math.isFinite(overtravel) || overtravel < 0.0)
      return "overtravel must be finite and non-negative";
    return null;
  }
}

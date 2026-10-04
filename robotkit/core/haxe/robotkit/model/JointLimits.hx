package robotkit.model;

import robotkit.model.EngineeringAssumptions.QuantityAssumption;

class JointLimits {
  public var assumptions:Array<QuantityAssumption> = [];
  public var lower:Float;
  public var upper:Float;
  public var velocity:Null<Float>;
  /** The hardware ceiling, when it supplies the effective velocity cap. */
  public var velocityLimiter:String = "";
  public var effort:Null<Float>;
  public var maxAcceleration:Null<Float>;
  /**
   * How far the joint can travel past `lower` and `upper` before it meets its
   * end stop, in joint units, as a machine's limit switch sits beyond its soft
   * limit. Commands stay within the limits; the runtime faults only past this.
   */
  public var overtravel:Float = 0.0;

  public function new(?lower:Float = 0.0, ?upper:Float = 0.0,
      ?velocity:Float, ?effort:Float,
      ?maxAcceleration:Float) {
    this.lower = lower;
    this.upper = upper;
    this.velocity = velocity;
    this.effort = effort;
    this.maxAcceleration = maxAcceleration;
  }

  public function copy():JointLimits {
    var result = new JointLimits(lower, upper, velocity, effort, maxAcceleration);
    result.assumptions = [for (value in assumptions) {quantity: value.quantity, label: value.label}];
    result.overtravel = overtravel;
    result.velocityLimiter = velocityLimiter;
    return result;
  }

  /** Require a stated or derived cap when a planner needs a finite limit. Zero remains a valid cap. */
  public function requireVelocity():Float {
    var value = velocity;
    if (value == null) throw "Joint has no velocity limit";
    return value;
  }

  public function requireEffort():Float {
    var value = effort;
    if (value == null) throw "Joint has no effort limit";
    return value;
  }

  public function requireAcceleration():Float {
    var value = maxAcceleration;
    if (value == null) throw "Joint has no acceleration limit";
    return value;
  }

  public function validate():Null<String> {
    if (lower > upper) return "lower limit exceeds upper limit";
    if (velocity != null && (!Math.isFinite(velocity) || velocity < 0.0)) return "velocity limit must be non-negative";
    if (effort != null && (!Math.isFinite(effort) || effort < 0.0)) return "effort limit must be non-negative";
    if (maxAcceleration != null && (!Math.isFinite(maxAcceleration) || maxAcceleration < 0.0))
      return "maximum acceleration must be finite and non-negative";
    if (!Math.isFinite(overtravel) || overtravel < 0.0)
      return "overtravel must be finite and non-negative";
    return null;
  }
}

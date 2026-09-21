package robot.model;

class JointLimits {
  public var lower:Float;
  public var upper:Float;
  public var velocity:Float;
  public var effort:Float;

  public function new(?lower:Float = 0.0, ?upper:Float = 0.0,
      ?velocity:Float = 0.0, ?effort:Float = 0.0) {
    this.lower = lower;
    this.upper = upper;
    this.velocity = velocity;
    this.effort = effort;
  }

  public function validate():Null<String> {
    if (lower > upper) return "lower limit exceeds upper limit";
    if (velocity < 0.0) return "velocity limit must be non-negative";
    if (effort < 0.0) return "effort limit must be non-negative";
    return null;
  }
}

package motionkit.planner;

/** Joint limits, optional per-span feed caps, and endpoint path speeds. */
class PathTimingLimits {
  public final maxVelocity:Array<Float>;
  public final maxAcceleration:Array<Float>;
  /** Empty means no authored feed caps; zero in one span means uncapped. */
  public final speedCaps:Array<Float>;
  public final startPathSpeed:Float;
  public final endPathSpeed:Float;

  public function new(maxVelocity:Array<Float>, maxAcceleration:Array<Float>,
      ?speedCaps:Array<Float>, ?startPathSpeed:Float = 0.0,
      ?endPathSpeed:Float = 0.0) {
    if (maxVelocity == null || maxAcceleration == null || maxVelocity.length == 0 ||
        maxVelocity.length != maxAcceleration.length)
      throw "Path timing needs matching per-joint limits";
    for (value in maxVelocity)
      if (!Math.isFinite(value) || value <= 0.0)
        throw "Path timing joint velocity limits must be finite and positive";
    for (value in maxAcceleration)
      if (!Math.isFinite(value) || value <= 0.0)
        throw "Path timing joint acceleration limits must be finite and positive";
    var caps = speedCaps == null ? [] : speedCaps.copy();
    for (value in caps)
      if (!Math.isFinite(value) || value < 0.0)
        throw "Path timing speed caps must be finite and non-negative";
    if (!Math.isFinite(startPathSpeed) || startPathSpeed < 0.0 ||
        !Math.isFinite(endPathSpeed) || endPathSpeed < 0.0)
      throw "Path timing endpoint speeds must be finite and non-negative";
    this.maxVelocity = maxVelocity.copy();
    this.maxAcceleration = maxAcceleration.copy();
    this.speedCaps = caps;
    this.startPathSpeed = startPathSpeed;
    this.endPathSpeed = endPathSpeed;
  }
}

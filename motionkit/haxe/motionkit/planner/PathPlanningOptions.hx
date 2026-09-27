package motionkit.planner;

/** Corner policy for geometric-path planning. */
class PathPlanningOptions {
  public final exactStop:Bool;
  public final blendTolerance:Float;
  public final maxBlendTurnAngleRadians:Float;

  public function new(?exactStop:Bool = true, ?blendTolerance:Float = 0.0,
      ?maxBlendTurnAngleRadians:Float = Math.PI * 5.0 / 6.0) {
    if (!Math.isFinite(blendTolerance) || blendTolerance < 0.0)
      throw "Path blend tolerance must be finite and non-negative";
    if (!Math.isFinite(maxBlendTurnAngleRadians) ||
        maxBlendTurnAngleRadians <= 0.0 || maxBlendTurnAngleRadians >= Math.PI)
      throw "Path maximum blend turn angle must be between zero and pi";
    this.exactStop = exactStop;
    this.blendTolerance = blendTolerance;
    this.maxBlendTurnAngleRadians = maxBlendTurnAngleRadians;
  }

  public static function exactStopMode():PathPlanningOptions
    return new PathPlanningOptions(true, 0.0);

  public static function blend(tolerance:Float,
      ?maxTurnAngleRadians:Float = Math.PI * 5.0 / 6.0):PathPlanningOptions
    return new PathPlanningOptions(false, tolerance, maxTurnAngleRadians);
}

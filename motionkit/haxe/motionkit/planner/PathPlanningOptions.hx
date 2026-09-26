package motionkit.planner;

/** Corner policy for geometric-path planning. */
class PathPlanningOptions {
  public final exactStop:Bool;
  public final blendTolerance:Float;

  public function new(?exactStop:Bool = true, ?blendTolerance:Float = 0.0) {
    if (!Math.isFinite(blendTolerance) || blendTolerance < 0.0)
      throw "Path blend tolerance must be finite and non-negative";
    this.exactStop = exactStop;
    this.blendTolerance = blendTolerance;
  }

  public static function exactStopMode():PathPlanningOptions
    return new PathPlanningOptions(true, 0.0);

  public static function blend(tolerance:Float):PathPlanningOptions
    return new PathPlanningOptions(false, tolerance);
}

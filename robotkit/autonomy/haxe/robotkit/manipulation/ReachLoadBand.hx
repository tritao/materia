package robotkit.manipulation;

/** A load rating valid through the specified flange distance from the base. */
class ReachLoadBand {
  public final maxReachMeters:Float;
  public final limits:RobotPayloadLimit;

  public function new(maxReachMeters:Float, limits:RobotPayloadLimit) {
    if (!Math.isFinite(maxReachMeters) || maxReachMeters <= 0 || limits == null)
      throw "Reach load band requires positive reach and payload limits";
    this.maxReachMeters = maxReachMeters;
    this.limits = limits;
  }
}

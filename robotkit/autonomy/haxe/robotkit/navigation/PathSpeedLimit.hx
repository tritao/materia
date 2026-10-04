package robotkit.navigation;

/** Maximum body speed allowed over one arc-length interval of a Path. */
class PathSpeedLimit {
  public final startDistanceMeters:Float;
  public final endDistanceMeters:Float;
  public final maximumSpeedMetersPerSecond:Float;

  public function new(startDistanceMeters:Float, endDistanceMeters:Float,
      maximumSpeedMetersPerSecond:Float) {
    if (!Math.isFinite(startDistanceMeters) || startDistanceMeters < 0.0 ||
        !Math.isFinite(endDistanceMeters) || endDistanceMeters <= startDistanceMeters ||
        !Math.isFinite(maximumSpeedMetersPerSecond) ||
        maximumSpeedMetersPerSecond <= 0.0)
      throw "Path speed limit requires a non-empty interval and positive finite speed";
    this.startDistanceMeters = startDistanceMeters;
    this.endDistanceMeters = endDistanceMeters;
    this.maximumSpeedMetersPerSecond = maximumSpeedMetersPerSecond;
  }
}

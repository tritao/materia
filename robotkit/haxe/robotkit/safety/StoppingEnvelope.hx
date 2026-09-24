package robotkit.safety;

/** Conservative one-dimensional stopping distance from speed, latency, and deceleration. */
class StoppingEnvelope {
  public final speedMetersPerSecond:Float;
  public final reactionTimeSeconds:Float;
  public final decelerationMetersPerSecondSquared:Float;
  public final distanceMeters:Float;

  public function new(speedMetersPerSecond:Float, reactionTimeSeconds:Float,
      decelerationMetersPerSecondSquared:Float) {
    if (!Math.isFinite(speedMetersPerSecond) || speedMetersPerSecond < 0.0 ||
        !Math.isFinite(reactionTimeSeconds) || reactionTimeSeconds < 0.0 ||
        !Math.isFinite(decelerationMetersPerSecondSquared) ||
        decelerationMetersPerSecondSquared <= 0.0)
      throw "Stopping envelope inputs are invalid";
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.reactionTimeSeconds = reactionTimeSeconds;
    this.decelerationMetersPerSecondSquared = decelerationMetersPerSecondSquared;
    distanceMeters = speedMetersPerSecond * reactionTimeSeconds +
      speedMetersPerSecond * speedMetersPerSecond /
      (2.0 * decelerationMetersPerSecondSquared);
  }
}

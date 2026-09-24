package robotkit.perception;

/** Circular obstacle observation in the detection's frame. */
class Obstacle {
  public final detection:Detection;
  public final radiusMeters:Float;

  public function new(detection:Detection, radiusMeters:Float) {
    if (detection == null || !Math.isFinite(radiusMeters) || radiusMeters <= 0.0)
      throw "Obstacle requires a detection and positive finite radius";
    this.detection = detection;
    this.radiusMeters = radiusMeters;
  }
}

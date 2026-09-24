package robotkit.perception;

/** Semantic pallet detection with its observed planar dimensions. */
class Pallet {
  public final detection:Detection;
  public final lengthMeters:Float;
  public final widthMeters:Float;
  public final heightMeters:Float;

  public function new(detection:Detection, lengthMeters:Float,
      widthMeters:Float, heightMeters:Float) {
    if (detection == null || !Math.isFinite(lengthMeters) || lengthMeters <= 0.0 ||
        !Math.isFinite(widthMeters) || widthMeters <= 0.0 ||
        !Math.isFinite(heightMeters) || heightMeters <= 0.0)
      throw "Pallet requires a detection and positive finite dimensions";
    this.detection = detection;
    this.lengthMeters = lengthMeters;
    this.widthMeters = widthMeters;
    this.heightMeters = heightMeters;
  }
}

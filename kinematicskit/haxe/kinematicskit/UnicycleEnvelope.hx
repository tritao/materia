package kinematicskit;

/** Wheel-derived bound |v| + |omega| b/2 <= groundSpeed, preserving commanded curvature. */
class UnicycleEnvelope {
  public final groundSpeed:Float;
  public final trackWidth:Float;

  public function new(groundSpeed:Float, trackWidth:Float) {
    if (!Math.isFinite(groundSpeed) || groundSpeed < 0 || !Math.isFinite(trackWidth) || trackWidth <= 0)
      throw "A wheel velocity envelope needs non-negative speed and positive track width";
    this.groundSpeed = groundSpeed;
    this.trackWidth = trackWidth;
  }

  public function pathSpeed(curvature:Float):Float
    return groundSpeed / (1 + Math.abs(curvature) * trackWidth / 2);

  public function constrain(linear:Float, angular:Float):{linear:Float, angular:Float} {
    var requested = Math.abs(linear) + Math.abs(angular) * trackWidth / 2;
    var scale = requested > groundSpeed ? groundSpeed / requested : 1.0;
    return {linear: linear * scale, angular: angular * scale};
  }
}

package robotkit.mobile;

/** Body-frame planar velocity: +X forward, +Y lateral, and +Z yaw rate. */
class Twist2 {
  public final linear:Float;
  public final angular:Float;
  public final lateral:Float;

  public function new(linear:Float = 0.0, angular:Float = 0.0, lateral:Float = 0.0) {
    if (!Math.isFinite(linear) || !Math.isFinite(angular) || !Math.isFinite(lateral))
      throw "Twist2 values must be finite";
    this.linear = linear;
    this.angular = angular;
    this.lateral = lateral;
  }

  public static function zero():Twist2 return new Twist2();
}

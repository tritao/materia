package robotkit.mobile;

/** Forward body speed in metres per second and yaw rate in radians per second. */
class Twist2 {
  public final linear:Float;
  public final angular:Float;

  public function new(linear:Float = 0.0, angular:Float = 0.0) {
    if (!Math.isFinite(linear) || !Math.isFinite(angular))
      throw "Twist2 values must be finite";
    this.linear = linear;
    this.angular = angular;
  }

  public static function zero():Twist2 return new Twist2();
}

package robotkit.spatial;

/** Immutable spatial velocity: linear (m/s) and angular (rad/s) parts, both expressed in the same frame. */
class Twist3 {
  public final linear:Vec3;
  public final angular:Vec3;

  public function new(linear:Vec3, angular:Vec3) {
    if (linear == null || angular == null)
      throw "Twist3 requires linear and angular components";
    this.linear = linear;
    this.angular = angular;
  }

  public static function zero():Twist3 return new Twist3(Vec3.zero(), Vec3.zero());
}

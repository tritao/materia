package robotkit.spatial;

/** Immutable 3D vector in meters (or the caller's consistent SI unit). */
class Vec3 {
  public final x:Float;
  public final y:Float;
  public final z:Float;

  public function new(x:Float = 0.0, y:Float = 0.0, z:Float = 0.0) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
      throw "Vec3 components must be finite";
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public static function zero():Vec3 return new Vec3();

  public function add(other:Vec3):Vec3 return new Vec3(x + other.x, y + other.y, z + other.z);

  public function sub(other:Vec3):Vec3 return new Vec3(x - other.x, y - other.y, z - other.z);

  public function scale(factor:Float):Vec3 return new Vec3(x * factor, y * factor, z * factor);

  public function negate():Vec3 return new Vec3(-x, -y, -z);

  public function dot(other:Vec3):Float return x * other.x + y * other.y + z * other.z;

  public function cross(other:Vec3):Vec3 return new Vec3(
    y * other.z - z * other.y,
    z * other.x - x * other.z,
    x * other.y - y * other.x
  );

  public function norm():Float return Math.sqrt(dot(this));

  /** Returns a unit vector; throws for a zero (or near-zero) vector. */
  public function normalized():Vec3 {
    var length = norm();
    if (!Math.isFinite(length) || length <= 1e-12)
      throw "Vec3 cannot be normalized: zero length";
    return scale(1.0 / length);
  }

  public function toArray():Array<Float> return [x, y, z];

  public static function fromArray(values:Array<Float>):Vec3 {
    if (values == null || values.length != 3)
      throw "Vec3 requires exactly three components";
    return new Vec3(values[0], values[1], values[2]);
  }
}

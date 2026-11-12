package motionkit.path;

/** A direction in 3D: finite and nonzero, stored as a unit vector, so two parallel directions are equal. */
struct Direction3 implements Equatable {
  public final x:Float;
  public final y:Float;
  public final z:Float;

  public function new(x:Float, y:Float, z:Float) {
    var norm = Math.sqrt(x * x + y * y + z * z);
    if (!Math.isFinite(norm) || norm <= 0) throw "A direction needs finite, nonzero components";
    this.x = x / norm;
    this.y = y / norm;
    this.z = z / norm;
  }

  public static function of(components:Array<Float>):Direction3 {
    if (components == null || components.length != 3) throw "A direction has three components";
    return new Direction3(components[0], components[1], components[2]);
  }

  public function components():Array<Float>
    return [x, y, z];
}

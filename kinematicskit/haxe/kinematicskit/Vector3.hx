package kinematicskit;

/** A 3D point or direction in the model's length unit. */
class Vector3 {
  public final x:Float;
  public final y:Float;
  public final z:Float;

  public function new(x:Float, y:Float, z:Float) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
}

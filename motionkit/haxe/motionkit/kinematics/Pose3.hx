package motionkit.kinematics;

/** Immutable position in metres and unit quaternion in xyzw order. */
class Pose3 {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public final qx:Float;
  public final qy:Float;
  public final qz:Float;
  public final qw:Float;

  public function new(?x:Float = 0.0, ?y:Float = 0.0, ?z:Float = 0.0,
      ?qx:Float = 0.0, ?qy:Float = 0.0, ?qz:Float = 0.0, ?qw:Float = 1.0) {
    for (value in [x, y, z, qx, qy, qz, qw])
      if (!Math.isFinite(value)) throw "Pose3 values must be finite";
    var normSquared = qx * qx + qy * qy + qz * qz + qw * qw;
    if (Math.abs(normSquared - 1.0) > 1e-6)
      throw "Pose3 rotation must be a unit quaternion";
    this.x = x;
    this.y = y;
    this.z = z;
    this.qx = qx;
    this.qy = qy;
    this.qz = qz;
    this.qw = qw;
  }

  public static function fromArrays(position:Array<Float>, rotation:Array<Float>):Pose3 {
    if (position == null || position.length != 3)
      throw "Pose3 position requires exactly three components";
    if (rotation == null || rotation.length != 4)
      throw "Pose3 rotation requires exactly four xyzw components";
    return new Pose3(position[0], position[1], position[2],
      rotation[0], rotation[1], rotation[2], rotation[3]);
  }

  public function positionArray():Array<Float> return [x, y, z];

  public function rotationArray():Array<Float> return [qx, qy, qz, qw];
}

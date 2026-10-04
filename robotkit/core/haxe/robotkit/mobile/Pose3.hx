package robotkit.mobile;

/** Immutable rigid pose in meters with an xyzw quaternion rotation. */
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
    var norm = qx * qx + qy * qy + qz * qz + qw * qw;
    if (Math.abs(norm - 1.0) > 0.000001)
      throw "Pose3 rotation must be a unit quaternion";
    this.x = x;
    this.y = y;
    this.z = z;
    this.qx = qx;
    this.qy = qy;
    this.qz = qz;
    this.qw = qw;
  }

  /** Composes a local pose into this pose's parent frame. */
  public function compose(local:Pose3):Pose3 {
    if (local == null) throw "Pose3 composition requires a local pose";
    var translation = rotate(local.x, local.y, local.z);
    var x = qw * local.qx + qx * local.qw + qy * local.qz - qz * local.qy;
    var y = qw * local.qy - qx * local.qz + qy * local.qw + qz * local.qx;
    var z = qw * local.qz + qx * local.qy - qy * local.qx + qz * local.qw;
    var w = qw * local.qw - qx * local.qx - qy * local.qy - qz * local.qz;
    var norm = Math.pow(x * x + y * y + z * z + w * w, 0.5);
    return new Pose3(this.x + translation.x, this.y + translation.y,
      this.z + translation.z, x / norm, y / norm, z / norm, w / norm);
  }

  /** Projects the position and local +X heading onto a Z-up planar frame. */
  public function planarPose():Pose2 {
    var sinYaw = 2.0 * (qw * qz + qx * qy);
    var cosYaw = 1.0 - 2.0 * (qy * qy + qz * qz);
    return new Pose2(x, y, PlanarMath.atan2(sinYaw, cosYaw));
  }

  /** Rotates a vector from this pose's local frame into its parent frame. */
  public function rotate(x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float} {
    var tx = 2.0 * (qy * z - qz * y);
    var ty = 2.0 * (qz * x - qx * z);
    var tz = 2.0 * (qx * y - qy * x);
    return {
      x: x + qw * tx + qy * tz - qz * ty,
      y: y + qw * ty + qz * tx - qx * tz,
      z: z + qw * tz + qx * ty - qy * tx
    };
  }
}

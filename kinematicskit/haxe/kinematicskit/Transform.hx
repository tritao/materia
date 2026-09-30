package kinematicskit;

/**
 * A rigid transform: translation plus a unit quaternion (x, y, z, w), in the
 * model's length unit. `a.compose(b)` is `a · b`, i.e. `b` expressed in
 * `a`'s frame. The formulas match `robotkit.spatial.Transform3` and
 * `materia.assembly.AssemblyFrames`, so compiled models reproduce both.
 */
class Transform {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public final qx:Float;
  public final qy:Float;
  public final qz:Float;
  public final qw:Float;

  public function new(x:Float, y:Float, z:Float, qx:Float, qy:Float, qz:Float, qw:Float) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.qx = qx;
    this.qy = qy;
    this.qz = qz;
    this.qw = qw;
  }

  public static function identity():Transform return new Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  public static function translation(x:Float, y:Float, z:Float):Transform
    return new Transform(x, y, z, 0.0, 0.0, 0.0, 1.0);

  /** Rotation of `angle` radians about a unit `axis`. */
  public static function axisAngle(ax:Float, ay:Float, az:Float, angle:Float):Transform {
    var s = Math.sin(angle * 0.5);
    return new Transform(0.0, 0.0, 0.0, ax * s, ay * s, az * s, Math.cos(angle * 0.5));
  }

  /**
   * Rejects non-finite values and a quaternion whose squared norm is more
   * than 1e-4 from one (the tolerance `AssemblyCodec.validateFrame` allows);
   * returns the transform with its quaternion normalized.
   */
  public static function checked(value:Transform, what:String):Transform {
    if (value == null) throw '$what requires a transform';
    if (!Math.isFinite(value.x) || !Math.isFinite(value.y) || !Math.isFinite(value.z) ||
        !Math.isFinite(value.qx) || !Math.isFinite(value.qy) || !Math.isFinite(value.qz) ||
        !Math.isFinite(value.qw))
      throw '$what must be finite';
    var squared = value.qx * value.qx + value.qy * value.qy + value.qz * value.qz + value.qw * value.qw;
    if (Math.abs(squared - 1.0) > 1e-4) throw '$what rotation must be a unit quaternion';
    var norm = Math.sqrt(squared);
    if (norm == 1.0) return value;
    return new Transform(value.x, value.y, value.z, value.qx / norm, value.qy / norm, value.qz / norm, value.qw / norm);
  }

  public function compose(child:Transform):Transform {
    var tx = 2.0 * (qy * child.z - qz * child.y);
    var ty = 2.0 * (qz * child.x - qx * child.z);
    var tz = 2.0 * (qx * child.y - qy * child.x);
    return new Transform(
      child.x + qw * tx + qy * tz - qz * ty + x,
      child.y + qw * ty + qz * tx - qx * tz + y,
      child.z + qw * tz + qx * ty - qy * tx + z,
      qw * child.qx + qx * child.qw + qy * child.qz - qz * child.qy,
      qw * child.qy - qx * child.qz + qy * child.qw + qz * child.qx,
      qw * child.qz + qx * child.qy - qy * child.qx + qz * child.qw,
      qw * child.qw - qx * child.qx - qy * child.qy - qz * child.qz);
  }

  public function inverse():Transform {
    var ix = -qx, iy = -qy, iz = -qz;
    var px = -x, py = -y, pz = -z;
    var tx = 2.0 * (iy * pz - iz * py);
    var ty = 2.0 * (iz * px - ix * pz);
    var tz = 2.0 * (ix * py - iy * px);
    return new Transform(
      px + qw * tx + iy * tz - iz * ty,
      py + qw * ty + iz * tx - ix * tz,
      pz + qw * tz + ix * ty - iy * tx,
      ix, iy, iz, qw);
  }

  /** Maps a point in this frame into the parent frame. */
  public function transformPoint(px:Float, py:Float, pz:Float):Vector3 {
    var tx = 2.0 * (qy * pz - qz * py);
    var ty = 2.0 * (qz * px - qx * pz);
    var tz = 2.0 * (qx * py - qy * px);
    return new Vector3(px + qw * tx + qy * tz - qz * ty + x,
      py + qw * ty + qz * tx - qx * tz + y,
      pz + qw * tz + qx * ty - qy * tx + z);
  }

  /** Rotates a free vector (no translation) into the parent frame. */
  public function transformVector(vx:Float, vy:Float, vz:Float):Vector3 {
    var tx = 2.0 * (qy * vz - qz * vy);
    var ty = 2.0 * (qz * vx - qx * vz);
    var tz = 2.0 * (qx * vy - qy * vx);
    return new Vector3(vx + qw * tx + qy * tz - qz * ty,
      vy + qw * ty + qz * tx - qx * tz,
      vz + qw * tz + qx * ty - qy * tx);
  }
}

package kinematicskit;

/** Rotation helpers on raw quaternion components (x, y, z, w). */
class Rotations {
  /**
   * Rotation vector (axis × angle, angle in [0, π]) of the normalized product
   * `a · b`. Below a half-angle sine of 1e-9 it is zero. This is the
   * orientation error RobotKit's DLS IK has always used, with `a` the target
   * and `b` the conjugate of the current orientation.
   */
  public static function logOfProduct(ax:Float, ay:Float, az:Float, aw:Float, bx:Float, by:Float, bz:Float,
      bw:Float):Vector3 {
    var x = aw * bx + ax * bw + ay * bz - az * by;
    var y = aw * by - ax * bz + ay * bw + az * bx;
    var z = aw * bz + ax * by - ay * bx + az * bw;
    var w = aw * bw - ax * bx - ay * by - az * bz;
    var norm = Math.sqrt(x * x + y * y + z * z + w * w);
    x /= norm; y /= norm; z /= norm; w /= norm;
    if (w < 0.0) { x = -x; y = -y; z = -z; w = -w; }
    var sinHalf = Math.sqrt(x * x + y * y + z * z);
    if (sinHalf < 1e-9) return new Vector3(0.0, 0.0, 0.0);
    var angle = 2.0 * Math.atan2(sinHalf, w);
    return new Vector3(x / sinHalf * angle, y / sinHalf * angle, z / sinHalf * angle);
  }

  /**
   * Rotation vector of `first⁻¹ · second`, in `first`'s frame: the shortest
   * rotation taking `first` to `second`. Matches CadKit's closure residual.
   */
  public static function relativeRotationVector(first:Transform, second:Transform):Vector3 {
    var x = first.qw * second.qx - first.qx * second.qw - first.qy * second.qz + first.qz * second.qy;
    var y = first.qw * second.qy + first.qx * second.qz - first.qy * second.qw - first.qz * second.qx;
    var z = first.qw * second.qz - first.qx * second.qy + first.qy * second.qx - first.qz * second.qw;
    var w = first.qw * second.qw + first.qx * second.qx + first.qy * second.qy + first.qz * second.qz;
    if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
    var sine = Math.sqrt(x * x + y * y + z * z);
    if (sine < 1e-12) return new Vector3(2 * x, 2 * y, 2 * z);
    var angle = 2 * Math.atan2(sine, w), scale = angle / sine;
    return new Vector3(x * scale, y * scale, z * scale);
  }

  /**
   * Two unit vectors perpendicular to the unit `axis` and to each other:
   * `u = axis × r / |axis × r|`, `v = axis × u / |axis × u|`, with `r` the X
   * axis unless `axis` is close to it (then Y).
   */
  public static function perpendicularBasis(x:Float, y:Float, z:Float):Array<Vector3> {
    var rx = 1.0, ry = 0.0, rz = 0.0;
    if (!(Math.abs(x) < 0.8)) { rx = 0.0; ry = 1.0; }
    var u = normalized(y * rz - z * ry, z * rx - x * rz, x * ry - y * rx);
    var v = normalized(y * u.z - z * u.y, z * u.x - x * u.z, x * u.y - y * u.x);
    return [u, v];
  }

  /**
   * Applies the inverse left Jacobian of SO(3) at rotation vector `phi` to
   * `v`: how the rotation vector of `R` changes when `R` is perturbed on the
   * left by a small rotation `v`.
   */
  public static function inverseLeftJacobian(phi:Vector3, v:Vector3):Vector3 {
    var theta2 = phi.x * phi.x + phi.y * phi.y + phi.z * phi.z;
    // [φ]× v and [φ]×² v.
    var cx = phi.y * v.z - phi.z * v.y, cy = phi.z * v.x - phi.x * v.z, cz = phi.x * v.y - phi.y * v.x;
    var ccx = phi.y * cz - phi.z * cy, ccy = phi.z * cx - phi.x * cz, ccz = phi.x * cy - phi.y * cx;
    var coefficient:Float;
    if (theta2 < 1e-8) coefficient = 1.0 / 12.0;
    else {
      var theta = Math.sqrt(theta2);
      var sine = Math.sin(theta);
      // At θ = π the (1 + cos θ) / sin θ term tends to zero.
      coefficient = sine < 1e-9 ? 1.0 / theta2 : 1.0 / theta2 - (1.0 + Math.cos(theta)) / (2.0 * theta * sine);
    }
    return new Vector3(v.x - 0.5 * cx + coefficient * ccx, v.y - 0.5 * cy + coefficient * ccy,
      v.z - 0.5 * cz + coefficient * ccz);
  }

  static function normalized(x:Float, y:Float, z:Float):Vector3 {
    var length = Math.sqrt(x * x + y * y + z * z);
    if (length < 1e-12) throw "Degenerate axis";
    return new Vector3(x / length, y / length, z / length);
  }
}

package kinematicskit;

/** Rotation helpers on raw quaternion components (x, y, z, w), writing into caller arrays. */
class Rotations {
  /**
   * Rotation vector (axis × angle, angle in [0, π]) of the normalized product
   * `a · b`, into `out[oi..oi+2]`. Below a half-angle sine of 1e-9 it is zero.
   * This is the orientation error RobotKit's DLS IK has always used, with `a`
   * the target and `b` the conjugate of the current orientation.
   */
  public static function logOfProduct(ax:Float, ay:Float, az:Float, aw:Float, bx:Float, by:Float, bz:Float,
      bw:Float, out:Array<Float>, oi:Int):Void {
    var x = aw * bx + ax * bw + ay * bz - az * by;
    var y = aw * by - ax * bz + ay * bw + az * bx;
    var z = aw * bz + ax * by - ay * bx + az * bw;
    var w = aw * bw - ax * bx - ay * by - az * bz;
    var norm = Math.sqrt(x * x + y * y + z * z + w * w);
    x /= norm; y /= norm; z /= norm; w /= norm;
    if (w < 0.0) { x = -x; y = -y; z = -z; w = -w; }
    var sinHalf = Math.sqrt(x * x + y * y + z * z);
    if (sinHalf < 1e-9) {
      out[oi] = 0.0; out[oi + 1] = 0.0; out[oi + 2] = 0.0;
      return;
    }
    var angle = 2.0 * Math.atan2(sinHalf, w);
    out[oi] = x / sinHalf * angle;
    out[oi + 1] = y / sinHalf * angle;
    out[oi + 2] = z / sinHalf * angle;
  }

  /**
   * Rotation vector of `first⁻¹ · second` in `first`'s frame (flat poses at
   * `fi`/`si`), into `out[oi..oi+2]`: the shortest rotation taking `first`
   * to `second`. Matches CadKit's closure residual.
   */
  public static function relativeRotationVector(first:Array<Float>, fi:Int, second:Array<Float>, si:Int,
      out:Array<Float>, oi:Int):Void {
    var ax = first[fi + 3], ay = first[fi + 4], az = first[fi + 5], aw = first[fi + 6];
    var bx = second[si + 3], by = second[si + 4], bz = second[si + 5], bw = second[si + 6];
    var x = aw * bx - ax * bw - ay * bz + az * by;
    var y = aw * by + ax * bz - ay * bw - az * bx;
    var z = aw * bz - ax * by + ay * bx - az * bw;
    var w = aw * bw + ax * bx + ay * by + az * bz;
    if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
    var sine = Math.sqrt(x * x + y * y + z * z);
    if (sine < 1e-12) {
      out[oi] = 2 * x; out[oi + 1] = 2 * y; out[oi + 2] = 2 * z;
      return;
    }
    var angle = 2 * Math.atan2(sine, w), scale = angle / sine;
    out[oi] = x * scale; out[oi + 1] = y * scale; out[oi + 2] = z * scale;
  }

  /**
   * Two unit vectors perpendicular to the unit `axis` and to each other,
   * into `out[oi..oi+5]` (u then v): `u = axis × r / |axis × r|`,
   * `v = axis × u / |axis × u|`, with `r` the X axis unless `axis` is close
   * to it (then Y).
   */
  public static function perpendicularBasis(x:Float, y:Float, z:Float, out:Array<Float>, oi:Int):Void {
    var rx = 1.0, ry = 0.0, rz = 0.0;
    if (!(Math.abs(x) < 0.8)) { rx = 0.0; ry = 1.0; }
    var ux = y * rz - z * ry, uy = z * rx - x * rz, uz = x * ry - y * rx;
    var length = Math.sqrt(ux * ux + uy * uy + uz * uz);
    if (length < 1e-12) throw "Degenerate axis";
    ux /= length; uy /= length; uz /= length;
    var vx = y * uz - z * uy, vy = z * ux - x * uz, vz = x * uy - y * ux;
    length = Math.sqrt(vx * vx + vy * vy + vz * vz);
    if (length < 1e-12) throw "Degenerate axis";
    out[oi] = ux; out[oi + 1] = uy; out[oi + 2] = uz;
    out[oi + 3] = vx / length; out[oi + 4] = vy / length; out[oi + 5] = vz / length;
  }

  /**
   * Applies the inverse left Jacobian of SO(3) at rotation vector `phi` to
   * `v`, into `out[oi..oi+2]`: how the rotation vector of `R` changes when
   * `R` is perturbed on the left by a small rotation `v`.
   */
  public static function inverseLeftJacobian(px:Float, py:Float, pz:Float, vx:Float, vy:Float, vz:Float,
      out:Array<Float>, oi:Int):Void {
    var theta2 = px * px + py * py + pz * pz;
    // [φ]× v and [φ]×² v.
    var cx = py * vz - pz * vy, cy = pz * vx - px * vz, cz = px * vy - py * vx;
    var ccx = py * cz - pz * cy, ccy = pz * cx - px * cz, ccz = px * cy - py * cx;
    var coefficient:Float;
    if (theta2 < 1e-8) coefficient = 1.0 / 12.0;
    else {
      var theta = Math.sqrt(theta2);
      var sine = Math.sin(theta);
      // At θ = π the (1 + cos θ) / sin θ term tends to zero.
      coefficient = sine < 1e-9 ? 1.0 / theta2 : 1.0 / theta2 - (1.0 + Math.cos(theta)) / (2.0 * theta * sine);
    }
    out[oi] = vx - 0.5 * cx + coefficient * ccx;
    out[oi + 1] = vy - 0.5 * cy + coefficient * ccy;
    out[oi + 2] = vz - 0.5 * cz + coefficient * ccz;
  }
}

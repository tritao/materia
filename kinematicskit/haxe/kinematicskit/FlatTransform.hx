package kinematicskit;

/**
 * Rigid-transform arithmetic on flat arrays, seven floats per transform
 * (x, y, z, qx, qy, qz, qw), so hot loops allocate nothing. Same formulas
 * as `Transform`.
 */
class FlatTransform {
  /** `out[oi..] = a[ai..] · b[bi..]`; `out` may alias either input. */
  public static function compose(a:Array<Float>, ai:Int, b:Array<Float>, bi:Int, out:Array<Float>, oi:Int):Void {
    var x = a[ai], y = a[ai + 1], z = a[ai + 2];
    var qx = a[ai + 3], qy = a[ai + 4], qz = a[ai + 5], qw = a[ai + 6];
    var bx = b[bi], by = b[bi + 1], bz = b[bi + 2];
    var bqx = b[bi + 3], bqy = b[bi + 4], bqz = b[bi + 5], bqw = b[bi + 6];
    var tx = 2.0 * (qy * bz - qz * by);
    var ty = 2.0 * (qz * bx - qx * bz);
    var tz = 2.0 * (qx * by - qy * bx);
    out[oi] = bx + qw * tx + qy * tz - qz * ty + x;
    out[oi + 1] = by + qw * ty + qz * tx - qx * tz + y;
    out[oi + 2] = bz + qw * tz + qx * ty - qy * tx + z;
    out[oi + 3] = qw * bqx + qx * bqw + qy * bqz - qz * bqy;
    out[oi + 4] = qw * bqy - qx * bqz + qy * bqw + qz * bqx;
    out[oi + 5] = qw * bqz + qx * bqy - qy * bqx + qz * bqw;
    out[oi + 6] = qw * bqw - qx * bqx - qy * bqy - qz * bqz;
  }

  /** `out[oi..oi+2]` = the rotation of `a[ai..]` applied to `(vx, vy, vz)`. */
  public static function rotate(a:Array<Float>, ai:Int, vx:Float, vy:Float, vz:Float, out:Array<Float>, oi:Int):Void {
    var qx = a[ai + 3], qy = a[ai + 4], qz = a[ai + 5], qw = a[ai + 6];
    var tx = 2.0 * (qy * vz - qz * vy);
    var ty = 2.0 * (qz * vx - qx * vz);
    var tz = 2.0 * (qx * vy - qy * vx);
    out[oi] = vx + qw * tx + qy * tz - qz * ty;
    out[oi + 1] = vy + qw * ty + qz * tx - qx * tz;
    out[oi + 2] = vz + qw * tz + qx * ty - qy * tx;
  }

  /** `out[oi..oi+2]` = the inverse rotation of `a[ai..]` applied to `(vx, vy, vz)`. */
  public static function rotateInverse(a:Array<Float>, ai:Int, vx:Float, vy:Float, vz:Float, out:Array<Float>,
      oi:Int):Void {
    var qx = -a[ai + 3], qy = -a[ai + 4], qz = -a[ai + 5], qw = a[ai + 6];
    var tx = 2.0 * (qy * vz - qz * vy);
    var ty = 2.0 * (qz * vx - qx * vz);
    var tz = 2.0 * (qx * vy - qy * vx);
    out[oi] = vx + qw * tx + qy * tz - qz * ty;
    out[oi + 1] = vy + qw * ty + qz * tx - qx * tz;
    out[oi + 2] = vz + qw * tz + qx * ty - qy * tx;
  }

  public static function write(value:Transform, out:Array<Float>, oi:Int):Void {
    out[oi] = value.x; out[oi + 1] = value.y; out[oi + 2] = value.z;
    out[oi + 3] = value.qx; out[oi + 4] = value.qy; out[oi + 5] = value.qz; out[oi + 6] = value.qw;
  }

  public static function read(values:Array<Float>, oi:Int):Transform
    return new Transform(values[oi], values[oi + 1], values[oi + 2], values[oi + 3], values[oi + 4],
      values[oi + 5], values[oi + 6]);
}

package robotkit.spatial;

/** Pure roll/pitch/yaw data, radians, ZYX intrinsic (see Quat.fromRollPitchYaw). */
typedef RollPitchYaw = {
  roll:Float,
  pitch:Float,
  yaw:Float
};

/**
 * Immutable unit quaternion, component order x, y, z, w, matching
 * ARCHITECTURE.md. The constructor normalizes its input so every live Quat
 * value is unit length.
 */
class Quat {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public final w:Float;

  public function new(x:Float = 0.0, y:Float = 0.0, z:Float = 0.0, w:Float = 1.0) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z) || !Math.isFinite(w))
      throw "Quat components must be finite";
    var norm = Math.sqrt(x * x + y * y + z * z + w * w);
    if (!Math.isFinite(norm) || norm <= 1e-12)
      throw "Quat cannot be built from a zero-length quaternion";
    this.x = x / norm;
    this.y = y / norm;
    this.z = z / norm;
    this.w = w / norm;
  }

  public static function identity():Quat return new Quat(0.0, 0.0, 0.0, 1.0);

  public static function fromAxisAngle(axis:Vec3, angleRadians:Float):Quat {
    var unit = axis.normalized();
    var half = angleRadians * 0.5;
    var s = Math.sin(half);
    return new Quat(unit.x * s, unit.y * s, unit.z * s, Math.cos(half));
  }

  /** Hamilton product: applies `other` after this rotation (this * other). */
  public function multiply(other:Quat):Quat return new Quat(
    w * other.x + x * other.w + y * other.z - z * other.y,
    w * other.y - x * other.z + y * other.w + z * other.x,
    w * other.z + x * other.y - y * other.x + z * other.w,
    w * other.w - x * other.x - y * other.y - z * other.z
  );

  public function conjugate():Quat return new Quat(-x, -y, -z, w);

  /** Normalized already; returns this value (kept for API symmetry with the plan). */
  public function normalize():Quat return new Quat(x, y, z, w);

  public function rotate(v:Vec3):Vec3 {
    var tx = 2.0 * (y * v.z - z * v.y);
    var ty = 2.0 * (z * v.x - x * v.z);
    var tz = 2.0 * (x * v.y - y * v.x);
    return new Vec3(
      v.x + w * tx + y * tz - z * ty,
      v.y + w * ty + z * tx - x * tz,
      v.z + w * tz + x * ty - y * tx
    );
  }

  public function dot(other:Quat):Float return x * other.x + y * other.y + z * other.z + w * other.w;

  /** Angle in radians between the rotations represented by this and `other`. */
  public function angularDistance(other:Quat):Float {
    var d = Math.abs(dot(other));
    if (d > 1.0) d = 1.0;
    return 2.0 * Math.acos(d);
  }

  public function slerp(other:Quat, t:Float):Quat {
    var cosHalfTheta = dot(other);
    var target = other;
    if (cosHalfTheta < 0.0) {
      target = new Quat(-other.x, -other.y, -other.z, -other.w);
      cosHalfTheta = -cosHalfTheta;
    }
    if (cosHalfTheta > 0.9995) {
      // Nearly parallel: fall back to a normalized linear blend.
      return new Quat(
        x + (target.x - x) * t,
        y + (target.y - y) * t,
        z + (target.z - z) * t,
        w + (target.w - w) * t
      );
    }
    var halfTheta = Math.acos(cosHalfTheta);
    var sinHalfTheta = Math.sqrt(1.0 - cosHalfTheta * cosHalfTheta);
    var ratioA = Math.sin((1.0 - t) * halfTheta) / sinHalfTheta;
    var ratioB = Math.sin(t * halfTheta) / sinHalfTheta;
    return new Quat(
      x * ratioA + target.x * ratioB,
      y * ratioA + target.y * ratioB,
      z * ratioA + target.z * ratioB,
      w * ratioA + target.w * ratioB
    );
  }

  /** Column-major 3x3 rotation matrix (9 values), matching ARCHITECTURE.md storage. */
  public function toRotationMatrix():Array<Float> {
    var xx = x * x, yy = y * y, zz = z * z;
    var xy = x * y, xz = x * z, yz = y * z;
    var wx = w * x, wy = w * y, wz = w * z;
    return [
      1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy),
      2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx),
      2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)
    ];
  }

  /** Builds from a column-major 3x3 rotation matrix (9 values). */
  public static function fromRotationMatrix(m:Array<Float>):Quat {
    if (m == null || m.length != 9)
      throw "Rotation matrix requires exactly nine components";
    function at(row:Int, col:Int):Float return m[col * 3 + row];
    var trace = at(0, 0) + at(1, 1) + at(2, 2);
    if (trace > 0.0) {
      var s = Math.sqrt(trace + 1.0) * 2.0;
      return new Quat(
        (at(2, 1) - at(1, 2)) / s,
        (at(0, 2) - at(2, 0)) / s,
        (at(1, 0) - at(0, 1)) / s,
        0.25 * s
      );
    } else if (at(0, 0) > at(1, 1) && at(0, 0) > at(2, 2)) {
      var s = Math.sqrt(1.0 + at(0, 0) - at(1, 1) - at(2, 2)) * 2.0;
      return new Quat(
        0.25 * s,
        (at(0, 1) + at(1, 0)) / s,
        (at(0, 2) + at(2, 0)) / s,
        (at(2, 1) - at(1, 2)) / s
      );
    } else if (at(1, 1) > at(2, 2)) {
      var s = Math.sqrt(1.0 + at(1, 1) - at(0, 0) - at(2, 2)) * 2.0;
      return new Quat(
        (at(0, 1) + at(1, 0)) / s,
        0.25 * s,
        (at(1, 2) + at(2, 1)) / s,
        (at(0, 2) - at(2, 0)) / s
      );
    } else {
      var s = Math.sqrt(1.0 + at(2, 2) - at(0, 0) - at(1, 1)) * 2.0;
      return new Quat(
        (at(0, 2) + at(2, 0)) / s,
        (at(1, 2) + at(2, 1)) / s,
        0.25 * s,
        (at(1, 0) - at(0, 1)) / s
      );
    }
  }

  /** Roll about X, then pitch about Y, then yaw about Z (R = Rz * Ry * Rx). */
  public static function fromRollPitchYaw(roll:Float, pitch:Float, yaw:Float):Quat {
    var qx = Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), roll);
    var qy = Quat.fromAxisAngle(new Vec3(0.0, 1.0, 0.0), pitch);
    var qz = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), yaw);
    return qz.multiply(qy).multiply(qx);
  }

  public function toRollPitchYaw():RollPitchYaw {
    var sinRollCosPitch = 2.0 * (w * x + y * z);
    var cosRollCosPitch = 1.0 - 2.0 * (x * x + y * y);
    var roll = Math.atan2(sinRollCosPitch, cosRollCosPitch);

    var sinPitch = 2.0 * (w * y - z * x);
    var pitch:Float;
    if (sinPitch >= 1.0) pitch = Math.PI * 0.5;
    else if (sinPitch <= -1.0) pitch = -Math.PI * 0.5;
    else pitch = Math.atan2(sinPitch, Math.sqrt(1.0 - sinPitch * sinPitch));

    var sinYawCosPitch = 2.0 * (w * z + x * y);
    var cosYawCosPitch = 1.0 - 2.0 * (y * y + z * z);
    var yaw = Math.atan2(sinYawCosPitch, cosYawCosPitch);
    return { roll: roll, pitch: pitch, yaw: yaw };
  }

  public function toArray():Array<Float> return [x, y, z, w];

  public static function fromArray(values:Array<Float>):Quat {
    if (values == null || values.length != 4)
      throw "Quat requires exactly four components (x, y, z, w)";
    return new Quat(values[0], values[1], values[2], values[3]);
  }
}

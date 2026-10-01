package motionkit.path;

import motionkit.kinematics.Pose3;

/**
  First and second derivatives of a pose with respect to path distance, in the
  base frame: linear in m/m and 1/m, angular in rad/m and rad/m².
**/
class PoseDerivatives {
  public final linear:Array<Float>;
  public final angular:Array<Float>;
  public final linearSecond:Array<Float>;
  public final angularSecond:Array<Float>;

  public function new(linear:Array<Float>, angular:Array<Float>,
      linearSecond:Array<Float>, angularSecond:Array<Float>) {
    for (vector in [linear, angular, linearSecond, angularSecond]) {
      if (vector == null || vector.length != 3) throw "Pose derivatives need three components each";
      for (value in vector) if (!Math.isFinite(value)) throw "Pose derivatives must be finite";
    }
    this.linear = linear;
    this.angular = angular;
    this.linearSecond = linearSecond;
    this.angularSecond = angularSecond;
  }

  /** Derivatives of a translation along `geometry` with a fixed orientation. */
  public static function ofGeometry(geometry:PathPrimitive, distance:Float):PoseDerivatives {
    var tangent = geometry.tangentAt(distance);
    var second = geometry.kind() == PathPrimitiveKind.Circular ?
      (cast geometry:CircularSegment).secondDerivativeAt(distance) : {
        // Planar primitives turn in their XY plane: the second derivative is
        // the curvature along the tangent's left normal.
        var curvature = geometry.curvatureAt(distance);
        [-tangent[1] * curvature, tangent[0] * curvature, 0.0];
      };
    return new PoseDerivatives(tangent, [0.0, 0.0, 0.0], second, [0.0, 0.0, 0.0]);
  }

  /**
    Derivatives of `primitive` by differences of its own geometry: central
    inside it and one-sided at its ends, so they never mix two primitives.
    For primitives with no closed form, such as interpolated orientations.
  **/
  public static function numeric(primitive:PosePrimitive, distance:Float):PoseDerivatives {
    var length = primitive.length();
    var step = Math.min(1e-5, length / 8.0);
    // Three points inside the primitive around `distance`.
    var middle = Math.max(step, Math.min(length - step, distance));
    var a = primitive.waypointAt(middle - step).pose;
    var b = primitive.waypointAt(middle).pose;
    var c = primitive.waypointAt(middle + step).pose;
    var offset = distance - middle;
    var linear:Array<Float> = [], linearSecond:Array<Float> = [];
    var pa = a.positionArray(), pb = b.positionArray(), pc = c.positionArray();
    for (axis in 0...3) {
      var second = (pa[axis] - 2.0 * pb[axis] + pc[axis]) / (step * step);
      linear.push((pc[axis] - pa[axis]) / (2.0 * step) + second * offset);
      linearSecond.push(second);
    }
    var before = rotationRate(a, b, step), after = rotationRate(b, c, step);
    var angularSecond = [for (axis in 0...3) (after[axis] - before[axis]) / step];
    var angular = [for (axis in 0...3)
      0.5 * (before[axis] + after[axis]) + angularSecond[axis] * offset];
    return new PoseDerivatives(linear, angular, linearSecond, angularSecond);
  }

  /** Base-frame rotation vector from `from` to `to`, per unit of `step`. */
  static function rotationRate(from:Pose3, to:Pose3, step:Float):Array<Float> {
    // to * conjugate(from): the rotation that carries `from` onto `to` in the base frame.
    var x = to.qw * -from.qx + to.qx * from.qw + to.qy * -from.qz - to.qz * -from.qy;
    var y = to.qw * -from.qy - to.qx * -from.qz + to.qy * from.qw + to.qz * -from.qx;
    var z = to.qw * -from.qz + to.qx * -from.qy - to.qy * -from.qx + to.qz * from.qw;
    var w = to.qw * from.qw - to.qx * -from.qx - to.qy * -from.qy - to.qz * -from.qz;
    if (w < 0.0) { x = -x; y = -y; z = -z; w = -w; }
    var sine = Math.sqrt(x * x + y * y + z * z);
    if (sine < 1e-15) return [0.0, 0.0, 0.0];
    var angle = 2.0 * Math.atan2(sine, w);
    return [x / sine * angle / step, y / sine * angle / step, z / sine * angle / step];
  }
}

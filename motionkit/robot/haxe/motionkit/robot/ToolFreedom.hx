package motionkit.robot;

import kinematicskit.FrameOrientation;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseMath;
import robotkit.manipulation.IkOptions;
import robotkit.spatial.Quat;

/** The hard tool task shared by pose IK, differential IK and path checks. */
class ToolFreedom {
  public final target:Pose3;
  public final orientation:FrameOrientation;
  public final orientationTolerance:Float;

  function new(target:Pose3, orientation:FrameOrientation, tolerance:Float) {
    this.target = target;
    this.orientation = orientation;
    orientationTolerance = tolerance;
  }

  public static function of(target:Pose3, policy:Null<OrientationPolicy>, tolerance:Float):ToolFreedom {
    if (target == null) throw "Tool freedom needs a target pose";
    return switch policy == null ? OrientationPolicy.Interpolated : policy {
      case Fixed | Interpolated: new ToolFreedom(target, Full, tolerance);
      case FreeAboutTool: new ToolFreedom(target, Axis(0, 0, 1), tolerance);
      case Free: new ToolFreedom(target, FrameOrientation.Free, tolerance);
      case Cone(axis, halfAngle):
        var unit = coneAxis(axis, halfAngle);
        // Rotate local Z onto the cone's axis, which is in the path frame.
        var qx = -unit[1], qy = unit[0], qw = 1.0 + unit[2];
        if (qw < 1e-12) { qx = 1.0; qy = 0.0; qw = 0.0; }
        var norm = Math.sqrt(qx*qx + qy*qy + qw*qw);
        new ToolFreedom(new Pose3(target.x, target.y, target.z, qx/norm, qy/norm, 0, qw/norm),
          Axis(0, 0, 1), halfAngle);
    };
  }

  public function options(tolerance:IkTolerance, ?preference:Pose3):IkOptions {
    var result = new IkOptions(tolerance.position, orientationTolerance, tolerance.maxIterations, tolerance.damping)
      .freedom(orientation);
    if (preference != null) switch orientation {
      case Full:
      default: result.preferringOrientation(new Quat(preference.qx, preference.qy, preference.qz, preference.qw));
    }
    return result;
  }

  public static function isFull(policy:Null<OrientationPolicy>):Bool
    return policy == null || switch policy {
      case Fixed | Interpolated: true;
      default: false;
    };

  public static function orientationError(actual:Pose3, desired:Pose3, policy:OrientationPolicy):Float {
    var task = of(desired, policy, 0.0);
    return switch task.orientation {
      case Full: PoseMath.angle(actual, desired);
      case Free: 0.0;
      case Axis(_, _, _): Math.max(0.0, axisAngle(toolAxis(actual), toolAxis(task.target)) -
        task.orientationTolerance);
    };
  }

  /** Rows that project a six-component twist onto the hard task at this pose. */
  public static function twistRows(actual:Pose3, policy:Null<OrientationPolicy>):Array<Array<Float>> {
    var task = of(actual, policy, 0.0);
    var rows = [[1.0, 0, 0, 0, 0, 0], [0.0, 1, 0, 0, 0, 0], [0.0, 0, 1, 0, 0, 0]];
    switch task.orientation {
      case Full:
        for (i in 0...3) rows.push([for (j in 0...6) j == i+3 ? 1.0 : 0.0]);
      case Free:
      case Axis(_, _, _):
        var z = toolAxis(actual);
        // Choose the coordinate least aligned with the tool axis.
        var coordinate = Math.abs(z[0]) <= Math.abs(z[1]) && Math.abs(z[0]) <= Math.abs(z[2]) ? 0
          : Math.abs(z[1]) <= Math.abs(z[2]) ? 1 : 2;
        var basis = [for (i in 0...3) (i == coordinate ? 1.0 : 0.0) - z[coordinate]*z[i]];
        var length = Math.sqrt(basis[0]*basis[0] + basis[1]*basis[1] + basis[2]*basis[2]);
        for (i in 0...3) basis[i] /= length;
        var other = [z[1]*basis[2] - z[2]*basis[1], z[2]*basis[0] - z[0]*basis[2],
          z[0]*basis[1] - z[1]*basis[0]];
        rows.push([0.0, 0, 0, basis[0], basis[1], basis[2]]);
        rows.push([0.0, 0, 0, other[0], other[1], other[2]]);
    }
    return rows;
  }

  /** Angular path rate that the hard task actually requires, in the path frame. */
  public static function requiredAngular(actual:Pose3, angular:Array<Float>, policy:Null<OrientationPolicy>):Array<Float> {
    if (isFull(policy)) return angular;
    switch policy {
      case Cone(_, _): return [0.0, 0.0, 0.0]; // Its axis is constant in the path frame.
      default:
    }
    var required = [0.0, 0.0, 0.0];
    for (row in twistRows(actual, policy)) {
      var value = 0.0;
      for (axis in 0...3) value += row[axis + 3]*angular[axis];
      for (axis in 0...3) required[axis] += row[axis + 3]*value;
    }
    return required;
  }

  public static function toolAxis(pose:Pose3):Array<Float>
    return [2.0*(pose.qx*pose.qz + pose.qw*pose.qy),
      2.0*(pose.qy*pose.qz - pose.qw*pose.qx),
      1.0 - 2.0*(pose.qx*pose.qx + pose.qy*pose.qy)];

  static function axisAngle(a:Array<Float>, b:Array<Float>):Float {
    var x = a[1]*b[2] - a[2]*b[1], y = a[2]*b[0] - a[0]*b[2], z = a[0]*b[1] - a[1]*b[0];
    return Math.atan2(Math.sqrt(x*x + y*y + z*z), a[0]*b[0] + a[1]*b[1] + a[2]*b[2]);
  }

  static function coneAxis(axis:Array<Float>, halfAngle:Float):Array<Float> {
    if (axis == null || axis.length != 3 || !Math.isFinite(halfAngle) || halfAngle < 0 || halfAngle > Math.PI)
      throw "Invalid orientation cone";
    var norm = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (!Math.isFinite(norm) || norm <= 0) throw "Invalid orientation cone axis";
    return [for (value in axis) value/norm];
  }
}

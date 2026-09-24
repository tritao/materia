package robotkit.mobile;

/** Immutable planar pose in metres and radians. */
class Pose2 {
  public final x:Float;
  public final y:Float;
  public final yaw:Float;

  public function new(x:Float = 0.0, y:Float = 0.0, yaw:Float = 0.0) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(yaw))
      throw "Pose2 values must be finite";
    this.x = x;
    this.y = y;
    this.yaw = wrapAngle(yaw);
  }

  /** Applies a pose expressed in this pose's local frame. */
  public function compose(local:Pose2):Pose2 {
    var c = Math.cos(yaw);
    var s = Math.sin(yaw);
    return new Pose2(x + c * local.x - s * local.y,
      y + s * local.x + c * local.y, yaw + local.yaw);
  }

  /** Returns this pose expressed relative to `origin`. */
  public function relativeTo(origin:Pose2):Pose2 {
    var dx = x - origin.x;
    var dy = y - origin.y;
    var c = Math.cos(origin.yaw);
    var s = Math.sin(origin.yaw);
    return new Pose2(c * dx + s * dy, -s * dx + c * dy, yaw - origin.yaw);
  }

  /** Integrates a constant body-frame velocity for a positive duration. */
  public function integrate(twist:Twist2, durationSeconds:Float):Pose2 {
    if (twist == null || !Math.isFinite(durationSeconds) || durationSeconds < 0.0)
      throw "Pose2 integration requires a finite non-negative duration";
    return integrateDisplacement(twist.linear * durationSeconds,
      twist.angular * durationSeconds);
  }

  /** Integrates forward distance and heading change along a constant-curvature arc. */
  public function integrateDisplacement(distance:Float, headingChange:Float):Pose2 {
    if (!Math.isFinite(distance) || !Math.isFinite(headingChange))
      throw "Pose2 displacement must be finite";
    var nextYaw = yaw + headingChange;
    var dx:Float;
    var dy:Float;
    if (Math.abs(headingChange) < 1e-9) {
      dx = distance * Math.cos(yaw);
      dy = distance * Math.sin(yaw);
    } else {
      var radius = distance / headingChange;
      dx = radius * (Math.sin(nextYaw) - Math.sin(yaw));
      dy = -radius * (Math.cos(nextYaw) - Math.cos(yaw));
    }
    return new Pose2(x + dx, y + dy, nextYaw);
  }

  public static function wrapAngle(value:Float):Float {
    var tau = Math.PI * 2.0;
    var wrapped = (value + Math.PI) % tau;
    if (wrapped < 0.0) wrapped += tau;
    return wrapped - Math.PI;
  }
}

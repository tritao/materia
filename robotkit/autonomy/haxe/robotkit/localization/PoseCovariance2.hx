package robotkit.localization;

/** Symmetric 3×3 covariance for planar x, y, and yaw. */
class PoseCovariance2 {
  public final xx:Float;
  public final xy:Float;
  public final xYaw:Float;
  public final yy:Float;
  public final yYaw:Float;
  public final yawYaw:Float;

  public function new(xx:Float = 0.0, xy:Float = 0.0, xYaw:Float = 0.0,
      yy:Float = 0.0, yYaw:Float = 0.0, yawYaw:Float = 0.0) {
    for (value in [xx, xy, xYaw, yy, yYaw, yawYaw])
      if (!Math.isFinite(value)) throw "Pose covariance values must be finite";
    if (xx < 0.0 || yy < 0.0 || yawYaw < 0.0)
      throw "Pose covariance variances must be non-negative";
    this.xx = xx;
    this.xy = xy;
    this.xYaw = xYaw;
    this.yy = yy;
    this.yYaw = yYaw;
    this.yawYaw = yawYaw;
  }

  public static function zero():PoseCovariance2 return new PoseCovariance2();
}

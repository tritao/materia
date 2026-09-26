package robotkit.perception;

import robotkit.spatial.Vec3;

/**
 * A fitted plane `normal . x = offset` (both expressed in the cloud's own
 * frame), plus the fit's inlier count and RMS residual.
 */
class PlaneEstimate {
  public final normal:Vec3;
  public final offset:Float;
  public final inlierCount:Int;
  public final rmsResidual:Float;

  public function new(normal:Vec3, offset:Float, inlierCount:Int, rmsResidual:Float) {
    if (normal == null) throw "PlaneEstimate requires a normal";
    if (!Math.isFinite(offset)) throw "PlaneEstimate offset must be finite";
    if (inlierCount < 0) throw "PlaneEstimate inlier count must be non-negative";
    if (!Math.isFinite(rmsResidual) || rmsResidual < 0.0) throw "PlaneEstimate RMS residual must be finite and non-negative";
    this.normal = normal;
    this.offset = offset;
    this.inlierCount = inlierCount;
    this.rmsResidual = rmsResidual;
  }

  /** Signed distance from `point` to this plane, positive on the normal's side. */
  public function signedDistance(point:Vec3):Float return normal.dot(point) - offset;
}

package robotkit.tool;

import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** One planned pose and acceleration of the workpiece centre in base axes.
 * The caller includes rotational acceleration terms in the COM acceleration. */
class SuctionMotionSample {
  public final baseTFlange:Transform3;
  public final baseAccelerationAtWorkpieceCom:Vec3;

  public function new(baseTFlange:Transform3, baseAccelerationAtWorkpieceCom:Vec3) {
    if (baseTFlange == null || baseAccelerationAtWorkpieceCom == null)
      throw "Suction motion sample requires a flange pose and COM acceleration";
    this.baseTFlange = baseTFlange;
    this.baseAccelerationAtWorkpieceCom = baseAccelerationAtWorkpieceCom;
  }
}

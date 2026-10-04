package robotkit.tool;

import robotkit.spatial.Vec3;

/** Force balance for one sealed cup over sampled workpiece motion. The model
 * treats normal suction and tangential friction separately; it does not model
 * leaks, cup deformation or load sharing among several cups. */
class SuctionCapacityChecker {
  public static inline var GRAVITY:Float = 9.80665;

  public static function checkPath(workpiece:WorkpieceLoad, grip:SuctionGrip,
      samples:Array<SuctionMotionSample>):SuctionCapacityResult {
    if (workpiece == null || grip == null || samples == null || samples.length == 0)
      throw "Suction check requires a workpiece, grip and motion samples";
    var capacity = grip.normalCapacityN();
    var allowedNormal = capacity / grip.safetyFactor;
    var worstNormal = Math.POSITIVE_INFINITY;
    var worstShear = Math.POSITIVE_INFINITY;
    var worstMoment:Null<Float> = grip.ratedMomentNm == null ? null : Math.POSITIVE_INFINITY;
    var workpieceAtFlange = workpiece.atFlange();
    var gravity = new Vec3(0, 0, -GRAVITY);
    for (index in 0...samples.length) {
      var sample = samples[index];
      if (sample == null) throw "Suction motion sample cannot be null";
      var cupPose = sample.baseTFlange.compose(grip.flangeTCup);
      var normal = cupPose.transformVector(new Vec3(0, 0, 1));
      var requiredForce = sample.baseAccelerationAtWorkpieceCom.sub(gravity)
        .scale(workpieceAtFlange.massKg);
      var axial = requiredForce.dot(normal);
      var tension = Math.max(0.0, -axial);
      var shear = requiredForce.sub(normal.scale(axial)).norm();
      var normalMargin = allowedNormal - tension;
      var shearMargin = grip.frictionCoefficient * Math.max(0.0, normalMargin) - shear;
      worstNormal = Math.min(worstNormal, normalMargin);
      worstShear = Math.min(worstShear, shearMargin);
      var cupToCom = sample.baseTFlange.transformPoint(workpieceAtFlange.centerOfMass)
        .sub(cupPose.translation);
      var moment = cupToCom.cross(requiredForce).norm();
      if (grip.ratedMomentNm != null) {
        var momentRating:Float = cast grip.ratedMomentNm;
        var previousWorst:Float = cast worstMoment;
        worstMoment = Math.min(previousWorst, momentRating / grip.safetyFactor - moment);
      }
      var reason:Null<String> = null;
      if (normalMargin < -1e-9) reason = "Normal suction force is insufficient";
      else if (shearMargin < -1e-9) reason = "Tangential friction force is insufficient";
      else if (grip.ratedMomentNm == null && moment > 1e-9)
        reason = "Cup overturning moment is unverified without a rating";
      else if (worstMoment != null && worstMoment < -1e-9)
        reason = "Cup overturning moment exceeds its rating";
      if (reason != null)
        return new SuctionCapacityResult(false, index, reason, capacity,
          worstNormal, worstShear, worstMoment);
    }
    return new SuctionCapacityResult(true, -1, null, capacity,
      worstNormal, worstShear, worstMoment);
  }
}

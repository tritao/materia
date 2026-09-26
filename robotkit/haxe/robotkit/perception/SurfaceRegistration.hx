package robotkit.perception;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.work.WorkSurface;
import robotkit.work.Provenance;
import robotkit.work.SourceKind;

/**
 * Result of `SurfaceRegistration.register`. `correction` is the minimal
 * rigid transform (`surface_T_correctedSurface`) that reconciles the design
 * plane (`z = 0` in the surface's own frame) with the observed plane: a
 * rotation tilting the design's `+Z` onto the fitted normal, about the axis
 * `(0, 0, 1) x normal` (identity when they already agree), and a translation
 * of `offset` along that normal — i.e. purely out-of-plane. In-plane
 * shift/rotation about the normal is not observable from a plane fit alone
 * and is left as identity. `registered` is the corrected `work` WorkSurface
 * (`null` when `accepted` is false).
 */
class SurfaceRegistrationResult {
  public final accepted:Bool;
  public final correction:Transform3;
  public final registered:Null<WorkSurface>;
  public final planeEstimate:PlaneEstimate;
  public final translationCorrection:Float;
  public final rotationCorrectionRadians:Float;
  public final rejectionReason:Null<String>;

  public function new(accepted:Bool, correction:Transform3, registered:Null<WorkSurface>,
      planeEstimate:PlaneEstimate, translationCorrection:Float, rotationCorrectionRadians:Float,
      ?rejectionReason:String) {
    this.accepted = accepted;
    this.correction = correction;
    this.registered = registered;
    this.planeEstimate = planeEstimate;
    this.translationCorrection = translationCorrection;
    this.rotationCorrectionRadians = rotationCorrectionRadians;
    this.rejectionReason = rejectionReason;
  }
}

/**
 * Registers a design `WorkSurface` against an observed point cloud: fits a
 * plane (seeded RANSAC, robust to outliers), derives the minimal correction
 * transform between the design plane and the fitted plane, and — unless the
 * correction exceeds the configured limits — derives a corrected `work`
 * `WorkSurface` that carries the design surface's boundary/exclusions (their
 * in-plane shape is unaffected by an out-of-plane correction) at the
 * corrected pose, with provenance pointing back at the design element.
 */
class SurfaceRegistration {
  public static function register(design:WorkSurface, cloud:PointCloud, seed:Int,
      ?maxIterations:Int = 500, ?inlierThreshold:Float = 0.01,
      ?maxTranslationCorrection:Float = 0.05, ?maxRotationCorrection:Float = 0.0872665):SurfaceRegistrationResult {
    if (design == null) throw "Surface registration requires a design work surface";
    if (cloud == null) throw "Surface registration requires an observed point cloud";
    if (cloud.frameId != design.surfaceFrameId)
      throw 'Surface registration requires the cloud to be expressed in surface frame "${design.surfaceFrameId}", got "${cloud.frameId}"';

    var designNormal = new Vec3(0.0, 0.0, 1.0);
    var estimate = PlaneFit.fitRansac(cloud.points, seed, maxIterations, inlierThreshold, designNormal);

    var normal = estimate.normal;
    var axis = designNormal.cross(normal);
    var axisLength = axis.norm();
    var cosAngle = designNormal.dot(normal);
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    var angle = Math.acos(cosAngle);

    var rotation = axisLength < 1e-9 ? Quat.identity() : Quat.fromAxisAngle(axis.normalized(), angle);
    var translation = normal.scale(estimate.offset);
    var correction = new Transform3(translation, rotation);

    var translationMagnitude = translation.norm();
    if (!(translationMagnitude <= maxTranslationCorrection) || !(angle <= maxRotationCorrection)) {
      var reason = 'registration correction (translation ${translationMagnitude}m, rotation ${angle}rad) exceeds configured limits (${maxTranslationCorrection}m, ${maxRotationCorrection}rad)';
      return new SurfaceRegistrationResult(false, correction, null, estimate, translationMagnitude, angle, reason);
    }

    var registeredSurfaceFrameId = design.surfaceFrameId + ":work";
    var registered = new WorkSurface(design.id, design.frameId,
      design.frame_T_surface.compose(correction), design.boundary, design.exclusions,
      design.tolerance, design.materialTag,
      new Provenance(design.provenance.designElementId, SourceKind.Work), registeredSurfaceFrameId);

    return new SurfaceRegistrationResult(true, correction, registered, estimate, translationMagnitude, angle);
  }
}

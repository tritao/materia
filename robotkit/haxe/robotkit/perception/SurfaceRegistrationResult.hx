package robotkit.perception;

import robotkit.spatial.Transform3;
import robotkit.work.WorkSurface;

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

package robotkit.localization;

/** Freshness, outlier, and recovery limits for pose fusion. */
class PoseFusionOptions {
  public final maxObservationAgeSeconds:Float;
  public final absoluteTimeoutSeconds:Float;
  public final maxPositionInnovationMeters:Float;
  public final maxYawInnovationRadians:Float;
  public final maxCorrectionStepMeters:Float;
  public final maxCorrectionStepRadians:Float;
  public final staleCovarianceGrowthPerSecond:Float;

  public function new(?maxObservationAgeSeconds:Float = 0.5,
      ?absoluteTimeoutSeconds:Float = 1.0,
      ?maxPositionInnovationMeters:Float = 2.0,
      ?maxYawInnovationRadians:Float = Math.PI / 4.0,
      ?maxCorrectionStepMeters:Float = 0.25,
      ?maxCorrectionStepRadians:Float = 0.15,
      ?staleCovarianceGrowthPerSecond:Float = 0.05) {
    for (value in [maxObservationAgeSeconds, absoluteTimeoutSeconds,
        maxPositionInnovationMeters, maxYawInnovationRadians,
        maxCorrectionStepMeters, maxCorrectionStepRadians,
        staleCovarianceGrowthPerSecond])
      if (!Math.isFinite(value) || value <= 0.0)
        throw "Pose fusion limits must be finite and positive";
    this.maxObservationAgeSeconds = maxObservationAgeSeconds;
    this.absoluteTimeoutSeconds = absoluteTimeoutSeconds;
    this.maxPositionInnovationMeters = maxPositionInnovationMeters;
    this.maxYawInnovationRadians = maxYawInnovationRadians;
    this.maxCorrectionStepMeters = maxCorrectionStepMeters;
    this.maxCorrectionStepRadians = maxCorrectionStepRadians;
    this.staleCovarianceGrowthPerSecond = staleCovarianceGrowthPerSecond;
  }
}

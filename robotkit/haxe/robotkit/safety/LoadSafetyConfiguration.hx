package robotkit.safety;

/** Tunable soft-limit reductions used by load-aware safety policy. */
class LoadSafetyConfiguration {
  public final payloadSpeedReduction:Float;
  public final payloadAccelerationReduction:Float;
  public final raisedForkSpeedReduction:Float;
  public final raisedForkAccelerationReduction:Float;
  public final unknownLoadSpeedScale:Float;
  public final unknownLoadAccelerationScale:Float;
  public final raisedForkThresholdMeters:Float;
  public final reactionTimeSeconds:Float;
  public final unknownPayloadMarginMeters:Float;

  public function new(?payloadSpeedReduction:Float = 0.4,
      ?payloadAccelerationReduction:Float = 0.5,
      ?raisedForkSpeedReduction:Float = 0.5,
      ?raisedForkAccelerationReduction:Float = 0.5,
      ?unknownLoadSpeedScale:Float = 0.35,
      ?unknownLoadAccelerationScale:Float = 0.35,
      ?raisedForkThresholdMeters:Float = 0.4,
      ?reactionTimeSeconds:Float = 0.25,
      ?unknownPayloadMarginMeters:Float = 0.25) {
    for (reduction in [payloadSpeedReduction, payloadAccelerationReduction,
        raisedForkSpeedReduction, raisedForkAccelerationReduction])
      if (!Math.isFinite(reduction) || reduction < 0.0 || reduction >= 1.0)
        throw "Safety reductions must be finite values in [0, 1)";
    for (scale in [unknownLoadSpeedScale, unknownLoadAccelerationScale])
      if (!Math.isFinite(scale) || scale <= 0.0 || scale > 1.0)
        throw "Unknown-load safety scales must be finite values in (0, 1]";
    if (!Math.isFinite(raisedForkThresholdMeters) || raisedForkThresholdMeters < 0.0 ||
        !Math.isFinite(reactionTimeSeconds) || reactionTimeSeconds < 0.0 ||
        !Math.isFinite(unknownPayloadMarginMeters) || unknownPayloadMarginMeters < 0.0)
      throw "Safety thresholds, reaction time, and footprint margin must be non-negative";
    this.payloadSpeedReduction = payloadSpeedReduction;
    this.payloadAccelerationReduction = payloadAccelerationReduction;
    this.raisedForkSpeedReduction = raisedForkSpeedReduction;
    this.raisedForkAccelerationReduction = raisedForkAccelerationReduction;
    this.unknownLoadSpeedScale = unknownLoadSpeedScale;
    this.unknownLoadAccelerationScale = unknownLoadAccelerationScale;
    this.raisedForkThresholdMeters = raisedForkThresholdMeters;
    this.reactionTimeSeconds = reactionTimeSeconds;
    this.unknownPayloadMarginMeters = unknownPayloadMarginMeters;
  }
}

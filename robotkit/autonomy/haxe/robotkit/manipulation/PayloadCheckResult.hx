package robotkit.manipulation;

/** First load-chart failure along a sampled joint path, or a safe result. */
class PayloadCheckResult {
  public final safe:Bool;
  public final firstFailureSegment:Int;
  public final segmentFraction:Float;
  public final reason:Null<String>;
  public final checkedSamples:Int;

  public function new(safe:Bool, firstFailureSegment:Int, segmentFraction:Float,
      reason:Null<String>, checkedSamples:Int) {
    this.safe = safe;
    this.firstFailureSegment = firstFailureSegment;
    this.segmentFraction = segmentFraction;
    this.reason = reason;
    this.checkedSamples = checkedSamples;
  }
}

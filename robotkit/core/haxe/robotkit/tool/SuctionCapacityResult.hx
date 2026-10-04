package robotkit.tool;

/** Worst margins across samples, with the first unsafe sample if any. */
class SuctionCapacityResult {
  public final safe:Bool;
  public final firstFailureIndex:Int;
  public final reason:Null<String>;
  public final normalCapacityN:Float;
  public final worstNormalMarginN:Float;
  public final worstShearMarginN:Float;
  /** Null when no moment rating was supplied. */
  public final worstMomentMarginNm:Null<Float>;

  public function new(safe:Bool, firstFailureIndex:Int, reason:Null<String>,
      normalCapacityN:Float, worstNormalMarginN:Float, worstShearMarginN:Float,
      worstMomentMarginNm:Null<Float>) {
    this.safe = safe;
    this.firstFailureIndex = firstFailureIndex;
    this.reason = reason;
    this.normalCapacityN = normalCapacityN;
    this.worstNormalMarginN = worstNormalMarginN;
    this.worstShearMarginN = worstShearMarginN;
    this.worstMomentMarginNm = worstMomentMarginNm;
  }
}

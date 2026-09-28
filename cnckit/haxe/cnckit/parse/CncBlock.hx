package cnckit.parse;

/** One source line after comments have been removed. */
class CncBlock {
  public final words:Array<CncWord>;
  public final span:CncSpan;

  public function new(words:Array<CncWord>, span:CncSpan) {
    this.words = words.copy();
    this.span = span;
  }
}

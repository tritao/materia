package cnckit.parse;

import toolpathkit.path.Provenance;
/** One source line after comments have been removed. */
class CncBlock {
  public final words:Array<CncWord>;
  public final span:Provenance;

  public function new(words:Array<CncWord>, span:Provenance) {
    this.words = words.copy();
    this.span = span;
  }
}

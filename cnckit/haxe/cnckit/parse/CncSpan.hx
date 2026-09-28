package cnckit.parse;

/** One-based source location. */
class CncSpan {
  public final line:Int;
  public final column:Int;
  public final length:Int;

  public function new(line:Int, column:Int, length:Int) {
    this.line = line;
    this.column = column;
    this.length = length;
  }
}

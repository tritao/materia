package stockkit;

import cnckit.CncTool;
import cnckit.parse.CncSpan;

/** One tool motion through the stock, with where it came from. */
class CutMove {
  public final tool:CncTool;
  public final motion:CutMotion;
  /** Rapid moves should not touch stock; cutting them is a diagnostic. */
  public final rapid:Bool;
  /** Index of the source operation in the program's op list. */
  public final opIndex:Int;
  public final span:CncSpan;

  public function new(tool:CncTool, motion:CutMotion, rapid:Bool,
      opIndex:Int, span:CncSpan) {
    if (tool == null || motion == null || span == null)
      throw "cut move needs a tool, motion and source span";
    this.tool = tool;
    this.motion = motion;
    this.rapid = rapid;
    this.opIndex = opIndex;
    this.span = span;
  }
}

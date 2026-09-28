package stockkit;

import toolpathkit.tool.Tool;
import toolpathkit.path.Provenance;

/** One tool motion through the stock, with where it came from. */
class CutMove {
  public final tool:Tool;
  public final motion:CutMotion;
  /** Rapid moves should not touch stock; cutting them is a diagnostic. */
  public final rapid:Bool;
  /** Index of the source operation in the program's op list. */
  public final opIndex:Int;
  public final span:Provenance;

  public function new(tool:Tool, motion:CutMotion, rapid:Bool,
      opIndex:Int, span:Provenance) {
    if (tool == null || motion == null || span == null)
      throw "cut move needs a tool, motion and source span";
    this.tool = tool;
    this.motion = motion;
    this.rapid = rapid;
    this.opIndex = opIndex;
    this.span = span;
  }
}

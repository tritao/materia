package toolpathkit.path;

import toolpathkit.tool.ToolLibrary;

/** Controller-independent operations and the tools referenced by changes. */
class ToolpathProgram {
  public final ops:Array<ToolpathOp>;
  public final tools:ToolLibrary;

  public function new(ops:Array<ToolpathOp>, tools:ToolLibrary) {
    if (ops == null || tools == null) throw "toolpath program needs operations and tools";
    this.ops = ops.copy();
    this.tools = tools;
  }
}

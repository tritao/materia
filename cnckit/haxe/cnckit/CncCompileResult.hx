package cnckit;

import toolpathkit.path.ToolpathOp;

/** G-code compilation result independent of motion execution. */
class CncCompileResult {
  public final ops:Array<ToolpathOp>;
  public final diagnostics:Array<CncDiagnostic>;

  public function new(ops:Array<ToolpathOp>, diagnostics:Array<CncDiagnostic>) {
    this.ops = ops.copy();
    this.diagnostics = diagnostics.copy();
  }
}

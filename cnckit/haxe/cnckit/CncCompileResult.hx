package cnckit;

import toolpathkit.path.ToolpathOp;
import motionkit.program.MotionProgram;

/** Detailed compilation result; program is null when no executable op survives. */
class CncCompileResult {
  public final program:Null<MotionProgram>;
  public final ops:Array<ToolpathOp>;
  public final sourceMap:CncSourceMap;
  public final diagnostics:Array<CncDiagnostic>;

  public function new(program:Null<MotionProgram>, ops:Array<ToolpathOp>,
      sourceMap:CncSourceMap, diagnostics:Array<CncDiagnostic>) {
    this.program = program;
    this.ops = ops.copy();
    this.sourceMap = sourceMap;
    this.diagnostics = diagnostics.copy();
  }
}

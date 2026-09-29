package cnckit;

import toolpathkit.path.ToolpathProgram;

/** G-code compilation result independent of motion execution. */
class CncCompileResult {
  public final program:ToolpathProgram;
  public final diagnostics:Array<CncDiagnostic>;

  public function new(program:ToolpathProgram, diagnostics:Array<CncDiagnostic>) {
    this.program = program;
    this.diagnostics = diagnostics.copy();
  }
}

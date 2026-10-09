package cnckit;

import toolpathkit.path.ToolpathProgram;

/** G-code compilation result independent of motion execution. */
class CncCompileResult {
  public final program:ToolpathProgram;
  public final diagnostics:Array<CncDiagnostic>;
  /** Did the program ask to home (`G28`)? Homing is the player's job, so it is not an operation. */
  public final homes:Bool;

  public function new(program:ToolpathProgram, diagnostics:Array<CncDiagnostic>, ?homes:Bool = false) {
    this.program = program;
    this.diagnostics = diagnostics.copy();
    this.homes = homes;
  }
}

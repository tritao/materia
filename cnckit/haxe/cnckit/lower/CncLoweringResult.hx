package cnckit.lower;

import cnckit.CncDiagnostic;
import cnckit.CncSourceMap;
import motionkit.program.MotionProgram;

class CncLoweringResult {
  public final program:Null<MotionProgram>;
  public final sourceMap:CncSourceMap;
  public final diagnostics:Array<CncDiagnostic>;

  public function new(program:Null<MotionProgram>, sourceMap:CncSourceMap,
      diagnostics:Array<CncDiagnostic>) {
    this.program = program;
    this.sourceMap = sourceMap;
    this.diagnostics = diagnostics;
  }
}

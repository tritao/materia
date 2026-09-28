package toolpathkit.motion;

import toolpathkit.motion.ToolpathSourceMap;
import motionkit.program.MotionProgram;

class ToolpathLoweringResult {
  public final program:Null<MotionProgram>;
  public final sourceMap:ToolpathSourceMap;
  public final diagnostics:Array<ToolpathDiagnostic>;

  public function new(program:Null<MotionProgram>, sourceMap:ToolpathSourceMap,
      diagnostics:Array<ToolpathDiagnostic>) {
    this.program = program;
    this.sourceMap = sourceMap;
    this.diagnostics = diagnostics;
  }
}

package toolpathkit.motion;

import toolpathkit.motion.ToolpathSourceMap;
import motionkit.program.MotionProgram;

class ToolpathLoweringResult {
  public final program:Null<MotionProgram>;
  public final sourceMap:ToolpathSourceMap;
  public final diagnostics:Array<ToolpathDiagnostic>;
  /** How many corners between moves the path stops at, by reason; tangent-continuous and rounded corners are not among them. */
  public final stops:Map<String, Int>;

  public function new(program:Null<MotionProgram>, sourceMap:ToolpathSourceMap,
      diagnostics:Array<ToolpathDiagnostic>, ?stops:Map<String, Int>) {
    this.program = program;
    this.sourceMap = sourceMap;
    this.diagnostics = diagnostics;
    this.stops = stops == null ? new Map() : stops;
  }
}

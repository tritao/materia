import cnckit.CncDiagnostic;
import cnckit.CncDiagnostic.CncSeverity;
import motionkit.program.MotionProgram;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.ToolpathOp;

/** Test harness for checking interpretation and execution together. */
class CncTestCompiler {
  public final rig:CncTestRig;
  public var warnings(default, null):Array<String> = [];

  public function new(rig:CncTestRig) this.rig = rig;

  public function compileDetailed(source:String):CncTestCompileResult {
    var ir = rig.compileDetailed(source);
    var diagnostics = ir.diagnostics.copy();
    var program:Null<MotionProgram> = null;
    var sourceMap = new ToolpathSourceMap();
    if (ir.program.ops.length > 0) try {
      var lowered = ToolpathMotion.lower(ir.program, rig.binding);
      program = lowered.program;
      sourceMap = lowered.sourceMap;
      for (warning in lowered.diagnostics)
        diagnostics.push(new CncDiagnostic(Warning, warning.code,
          warning.provenance, warning.message));
    } catch (error:Dynamic) {
      diagnostics.push(new CncDiagnostic(Error, "CNC_LOWER",
        new toolpathkit.path.Provenance(1, 1, 0), Std.string(error)));
    }
    diagnostics.sort(function(a, b) {
      if (a.span.line != b.span.line) return a.span.line - b.span.line;
      return a.span.column - b.span.column;
    });
    warnings = [for (diagnostic in diagnostics)
      if (diagnostic.severity == Warning)
        'G-code line ${diagnostic.span.line}: ${diagnostic.message}'];
    return new CncTestCompileResult(program, ir.program.ops, sourceMap, diagnostics);
  }

  public function compile(source:String):MotionProgram {
    var result = compileDetailed(source);
    for (diagnostic in result.diagnostics)
      if (diagnostic.severity == Error) throw diagnostic.toString();
    if (result.program == null) throw "G-code contains no executable motion or barrier";
    return result.program;
  }
}

class CncTestCompileResult {
  public final program:Null<MotionProgram>;
  public final ops:Array<ToolpathOp>;
  public final sourceMap:ToolpathSourceMap;
  public final diagnostics:Array<CncDiagnostic>;

  public function new(program:Null<MotionProgram>, ops:Array<ToolpathOp>,
      sourceMap:ToolpathSourceMap, diagnostics:Array<CncDiagnostic>) {
    this.program = program;
    this.ops = ops;
    this.sourceMap = sourceMap;
    this.diagnostics = diagnostics;
  }
}

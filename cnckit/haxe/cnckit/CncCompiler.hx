package cnckit;

import cnckit.CncDiagnostic.CncSeverity;
import cnckit.interp.CncInterpreter;
import cnckit.lower.CncLowering;
import cnckit.parse.CncLexer;
import cnckit.parse.CncSpan;
import motionkit.program.MotionProgram;

/** Compatibility facade over lexing, modal interpretation, and MotionKit lowering. */
class CncCompiler {
  public final machine:CncMachine;
  public var warnings(default, null):Array<String> = [];

  public function new(machine:CncMachine) {
    if (machine == null) throw "CNC compiler needs a machine";
    this.machine = machine;
  }

  /** Returns all diagnostics and the surviving preview/lowered operations. */
  public function compileDetailed(source:String):CncCompileResult {
    warnings = [];
    if (source == null) {
      var error = new CncDiagnostic(Error, "CNC_NULL_SOURCE",
        new CncSpan(1, 1, 0), "CNC source must not be null");
      return new CncCompileResult(null, [], new CncSourceMap(), [error]);
    }
    var parsed = CncLexer.parse(source);
    var interpreter = new CncInterpreter(machine);
    var interpreted = interpreter.interpret(parsed.blocks);
    var compensated = CncCompensator.resolve(interpreted);
    var ops = compensated.ops;
    var diagnostics = parsed.diagnostics.concat(interpreter.diagnostics);
    diagnostics = diagnostics.concat(compensated.diagnostics);
    diagnostics = diagnostics.concat(CncTravelChecks.check(machine, ops));
    var program:Null<MotionProgram> = null;
    var sourceMap = new CncSourceMap();
    try {
      var lowered = new CncLowering(machine).lower(ops);
      program = lowered.program;
      sourceMap = lowered.sourceMap;
      diagnostics = diagnostics.concat(lowered.diagnostics);
    } catch (error:Dynamic) {
      diagnostics.push(new CncDiagnostic(Error, "CNC_LOWER",
        new CncSpan(1, 1, 0), Std.string(error)));
    }
    if (program == null && diagnostics.length == 0)
      diagnostics.push(new CncDiagnostic(Error, "CNC_EMPTY",
        new CncSpan(1, 1, 0), "G-code contains no executable motion or barrier"));
    diagnostics.sort(function(a, b) {
      if (a.span.line != b.span.line) return a.span.line - b.span.line;
      return a.span.column - b.span.column;
    });
    warnings = [for (diagnostic in diagnostics)
      if (diagnostic.severity == Warning)
        'G-code line ${diagnostic.span.line}: ${diagnostic.message}'];
    return new CncCompileResult(program, ops, sourceMap, diagnostics);
  }

  /** Legacy entry point: throw the first error so existing MotionKit callers stay stable. */
  public function compile(source:String):MotionProgram {
    if (source == null) throw "CNC source must not be null";
    var result = compileDetailed(source);
    for (diagnostic in result.diagnostics)
      if (diagnostic.severity == Error) throw diagnostic.toString();
    if (result.program == null) throw "G-code contains no executable motion or barrier";
    return result.program;
  }
}

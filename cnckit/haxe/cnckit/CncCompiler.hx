package cnckit;

import cnckit.CncDiagnostic.CncSeverity;
import cnckit.interp.CncInterpreter;
import cnckit.parse.CncLexer;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.GeometryOffset;
import toolpathkit.setup.TravelEnvelope;

/** Parses and interprets G-code into the shared toolpath format. */
class CncCompiler {
  public final machine:CncMachine;
  public var warnings(default, null):Array<String> = [];

  public function new(machine:CncMachine) {
    if (machine == null) throw "CNC compiler needs a machine";
    this.machine = machine;
  }

  /** Returns diagnostics and the surviving toolpath operations. */
  public function compileDetailed(source:String):CncCompileResult {
    warnings = [];
    if (source == null) {
      var error = new CncDiagnostic(Error, "CNC_NULL_SOURCE",
        new Provenance(1, 1, 0), "CNC source must not be null");
      return new CncCompileResult([], [error]);
    }
    var parsed = CncLexer.parse(source);
    var interpreter = new CncInterpreter(machine);
    var interpreted = interpreter.interpret(parsed.blocks);
    var compensated = CncCompensator.resolve(interpreted);
    var machineOps = compensated.ops;
    var diagnostics = parsed.diagnostics.concat(interpreter.diagnostics);
    diagnostics = diagnostics.concat(compensated.diagnostics);
    diagnostics = diagnostics.concat([for (violation in
      TravelEnvelope.check(machine.travelLower, machine.travelUpper, machineOps))
      new CncDiagnostic(Error, "CNC_TRAVEL", violation.provenance,
        violation.message())]);
    var ops:Array<ToolpathOp> = [];
    var offset = machine.controller.workOffset(54);
    for (op in machineOps) switch op {
      case SetSetup(id, _):
        offset = machine.controller.workOffset(Std.parseInt(id) + 53);
        ops.push(op);
      case Move(kind, geometry, feed, tolerance, provenance):
        ops.push(ToolpathOp.Move(kind, GeometryOffset.translate(geometry,
          [-offset[0], -offset[1], -offset[2]]), feed, tolerance, provenance));
      case _: ops.push(op);
    }
    if (ops.length == 0 && diagnostics.length == 0)
      diagnostics.push(new CncDiagnostic(Error, "CNC_EMPTY",
        new Provenance(1, 1, 0), "G-code contains no executable motion or barrier"));
    diagnostics.sort(function(a, b) {
      if (a.span.line != b.span.line) return a.span.line - b.span.line;
      return a.span.column - b.span.column;
    });
    warnings = [for (diagnostic in diagnostics)
      if (diagnostic.severity == Warning)
        'G-code line ${diagnostic.span.line}: ${diagnostic.message}'];
    return new CncCompileResult(ops, diagnostics);
  }
}

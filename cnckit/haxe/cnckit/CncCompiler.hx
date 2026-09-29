package cnckit;

import cnckit.CncDiagnostic.CncSeverity;
import cnckit.interp.CncInterpreter;
import cnckit.parse.CncLexer;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.GeometryOffset;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.setup.TravelEnvelope;

/** Parses and interprets G-code into the shared toolpath format. */
class CncCompiler {
  /** Returns diagnostics and the surviving toolpath operations. */
  public static function compileDetailed(source:String, controller:CncController,
      ?start:Point3, ?travel:TravelEnvelope):CncCompileResult {
    if (controller == null) throw "CNC compiler needs a controller";
    var initial = start == null ? new Point3(0.0, 0.0, 0.0) : start;
    if (!Math.isFinite(initial.x) || !Math.isFinite(initial.y) ||
        !Math.isFinite(initial.z)) throw "CNC start needs finite XYZ";
    var setups = [setupFor(controller, "1")];
    if (source == null) {
      var error = new CncDiagnostic(Error, "CNC_NULL_SOURCE",
        new Provenance(1, 1, 0), "CNC source must not be null");
      return new CncCompileResult(new ToolpathProgram([], controller.toolLibrary, setups), [error]);
    }
    var parsed = CncLexer.parse(source);
    var interpreter = new CncInterpreter(controller, initial);
    var interpreted = interpreter.interpret(parsed.blocks);
    var compensated = CncCompensator.resolve(interpreted);
    var machineOps = compensated.ops;
    var diagnostics = parsed.diagnostics.concat(interpreter.diagnostics);
    diagnostics = diagnostics.concat(compensated.diagnostics);
    var known = new Map<String, Bool>();
    known.set("1", true);
    for (op in machineOps) switch op {
      case SetSetup(id, _):
        if (!known.exists(id)) {
          setups.push(setupFor(controller, id));
          known.set(id, true);
        }
      case _:
    }
    var ops:Array<ToolpathOp> = [];
    var offset = controller.workOffset(54);
    for (op in machineOps) switch op {
      case SetSetup(id, _):
        offset = controller.workOffset(controller.gCodeForSetup(id));
        ops.push(op);
      case Move(kind, geometry, feed, tolerance, provenance):
        ops.push(ToolpathOp.Move(kind, GeometryOffset.translate(geometry,
          [-offset[0], -offset[1], -offset[2]]), feed, tolerance, provenance));
      case _: ops.push(op);
    }
    if (travel != null)
      for (violation in travel.check(new ToolpathProgram(ops,
          controller.toolLibrary, setups)))
        diagnostics.push(new CncDiagnostic(Error, "CNC_TRAVEL",
          violation.provenance, violation.message()));
    if (ops.length == 0 && diagnostics.length == 0)
      diagnostics.push(new CncDiagnostic(Error, "CNC_EMPTY",
        new Provenance(1, 1, 0), "G-code contains no executable motion or barrier"));
    diagnostics.sort(function(a, b) {
      if (a.span.line != b.span.line) return a.span.line - b.span.line;
      return a.span.column - b.span.column;
    });
    return new CncCompileResult(new ToolpathProgram(ops, controller.toolLibrary,
      setups), diagnostics);
  }

  static function setupFor(controller:CncController, id:String):Setup {
    var offset = controller.workOffset(controller.gCodeForSetup(id));
    return new Setup(id, new Point3(offset[0], offset[1], offset[2]));
  }
}

package toolpathkit.motion;

import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.GeometryOffset;
import toolpathkit.path.ToolpathFrame;

/**
  Entry point for executing authored toolpaths on a bound machine. Moves are
  lowered at the machine's controlled point: work origin plus G43 length.
**/
class ToolpathMotion {
  public static function lower(program:ToolpathProgram,
      binding:MachineBinding):ToolpathLoweringResult {
    if (program == null || binding == null)
      throw "toolpath execution needs a program and a machine binding";
    var machineOps:Array<ToolpathOp> = [];
    var frame = ToolpathFrame.of(program);
    for (op in program.ops) {
      frame.advance(op);
      switch op {
        case Move(kind, geometry, feed, tolerance, provenance):
          machineOps.push(ToolpathOp.Move(kind,
            GeometryOffset.translate(geometry, frame.toMachine()), feed,
            tolerance, provenance));
        case MachineMove(kind, geometry, feed, tolerance, provenance):
          machineOps.push(ToolpathOp.Move(kind, geometry, feed, tolerance,
            provenance));
        case _: machineOps.push(op);
      }
    }
    var violations = binding.travel == null ? [] : binding.travel.check(program);
    if (violations.length > 0) {
      var first = violations[0];
      throw 'toolpath operation ${first.provenance.operationId == null ? first.provenance.line : first.provenance.operationId}: ${first.message()}';
    }
    return new ToolpathLowering(binding).lower(machineOps);
  }
}

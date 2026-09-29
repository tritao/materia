package toolpathkit.motion;

import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.GeometryOffset;
import toolpathkit.setup.Setup;

/** Entry point for executing authored toolpaths on a bound machine. */
class ToolpathMotion {
  public static function lower(program:ToolpathProgram,
      binding:MachineBinding):ToolpathLoweringResult {
    if (program == null || binding == null)
      throw "toolpath execution needs a program and a machine binding";
    var machineOps:Array<ToolpathOp> = [];
    var active = program.setups[0];
    var offset = [active.workOrigin.x, active.workOrigin.y, active.workOrigin.z];
    for (op in program.ops) switch op {
      case SetSetup(id, _):
        var next:Null<Setup> = null;
        for (setup in program.setups) if (setup.id == id) next = setup;
        if (next == null) throw 'Unknown setup $id';
        offset = [next.workOrigin.x, next.workOrigin.y, next.workOrigin.z];
        machineOps.push(op);
      case Move(kind, geometry, feed, tolerance, provenance):
        machineOps.push(ToolpathOp.Move(kind,
          GeometryOffset.translate(geometry, offset), feed, tolerance,
          provenance));
      case MachineMove(kind, geometry, feed, tolerance, provenance):
        machineOps.push(ToolpathOp.Move(kind, geometry, feed, tolerance,
          provenance));
      case _: machineOps.push(op);
    }
    var violations = binding.travel == null ? [] : binding.travel.check(program);
    if (violations.length > 0) {
      var first = violations[0];
      throw 'toolpath operation ${first.provenance.operationId == null ? first.provenance.line : first.provenance.operationId}: ${first.message()}';
    }
    return new ToolpathLowering(binding).lower(machineOps);
  }
}

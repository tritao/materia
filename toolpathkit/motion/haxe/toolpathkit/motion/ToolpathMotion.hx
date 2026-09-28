package toolpathkit.motion;

import toolpathkit.path.ToolpathOp;
import toolpathkit.path.GeometryOffset;
import toolpathkit.setup.TravelEnvelope;

/** Entry point for executing authored toolpaths on a bound machine. */
class ToolpathMotion {
  public static function lower(ops:Array<ToolpathOp>,
      binding:MachineBinding):ToolpathLoweringResult {
    if (ops == null || binding == null)
      throw "toolpath execution needs operations and a machine binding";
    var machineOps:Array<ToolpathOp> = [];
    var offset = binding.setupOffset("1");
    for (op in ops) switch op {
      case SetSetup(id, _):
        offset = binding.setupOffset(id);
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
    var violations = TravelEnvelope.check(binding.travelLower,
      binding.travelUpper, machineOps);
    if (violations.length > 0) {
      var first = violations[0];
      throw 'toolpath operation ${first.provenance.operationId == null ? first.provenance.line : first.provenance.operationId}: ${first.message()}';
    }
    return new ToolpathLowering(binding).lower(machineOps);
  }
}

package toolpathkit.motion;

import toolpathkit.path.ToolpathOp;
import toolpathkit.setup.TravelEnvelope;

/** Entry point for executing authored toolpaths on a bound machine. */
class ToolpathMotion {
  public static function lower(ops:Array<ToolpathOp>,
      binding:MachineBinding):ToolpathLoweringResult {
    if (ops == null || binding == null)
      throw "toolpath execution needs operations and a machine binding";
    var violations = TravelEnvelope.check(binding.travelLower,
      binding.travelUpper, ops);
    if (violations.length > 0) {
      var first = violations[0];
      throw 'toolpath operation ${first.provenance.operationId == null ? first.provenance.line : first.provenance.operationId}: ${first.message()}';
    }
    return new ToolpathLowering(binding).lower(ops);
  }
}

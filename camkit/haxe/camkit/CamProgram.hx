package camkit;

import cnckit.CncCompileResult;
import cnckit.CncMachine;
import cnckit.CncTool;
import cnckit.CncTravelChecks;
import cnckit.ir.CncOp;
import cnckit.lower.CncLowering;

/** CNC IR produced directly by CAM, with operation numbers as source spans. */
class CamProgram {
  public final ops:Array<CncOp>;
  /** Every tool the program changes to, one per tool number. */
  public final tools:Array<CncTool>;

  public function new(ops:Array<CncOp>, ?tools:Array<CncTool>) {
    if (ops == null || ops.length == 0) throw "CAM program needs operations";
    this.ops = ops.copy();
    this.tools = tools == null ? [] : tools.copy();
  }

  public function tool(number:Int):CncTool {
    for (tool in tools) if (tool.number == number) return tool;
    throw 'CAM program has no tool $number';
  }

  public function lower(machine:CncMachine):CncCompileResult {
    var diagnostics = CncTravelChecks.check(machine, ops);
    var lowered = new CncLowering(machine).lower(ops);
    return new CncCompileResult(lowered.program, ops, lowered.sourceMap,
      diagnostics.concat(lowered.diagnostics));
  }
}

package camkit;

import toolpathkit.tool.Tool;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Provenance;
import toolpathkit.setup.Setup;

/** CNC IR produced directly by CAM, with operation numbers as source spans. */
class CamProgram {
  public final ops:Array<ToolpathOp>;
  /** Every tool the program changes to, one per tool number. */
  public final tools:Array<Tool>;
  public final setup:Null<Setup>;

  public function new(ops:Array<ToolpathOp>, ?tools:Array<Tool>, ?setup:Setup) {
    if (ops == null || ops.length == 0) throw "CAM program needs operations";
    this.ops = ops.copy();
    this.setup = setup;
    if (setup != null)
      this.ops.unshift(ToolpathOp.SetSetup(setup.id, Provenance.cam(0)));
    this.tools = tools == null ? [] : tools.copy();
  }

  public function tool(number:Int):Tool {
    for (tool in tools) if (tool.number == number) return tool;
    throw 'CAM program has no tool $number';
  }

}

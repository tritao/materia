package camkit;

import toolpathkit.tool.Tool;
import toolpathkit.path.ToolpathOp;

/** CNC IR produced directly by CAM, with operation numbers as source spans. */
class CamProgram {
  public final ops:Array<ToolpathOp>;
  /** Every tool the program changes to, one per tool number. */
  public final tools:Array<Tool>;

  public function new(ops:Array<ToolpathOp>, ?tools:Array<Tool>) {
    if (ops == null || ops.length == 0) throw "CAM program needs operations";
    this.ops = ops.copy();
    this.tools = tools == null ? [] : tools.copy();
  }

  public function tool(number:Int):Tool {
    for (tool in tools) if (tool.number == number) return tool;
    throw 'CAM program has no tool $number';
  }

}

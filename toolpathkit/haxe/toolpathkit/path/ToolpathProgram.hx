package toolpathkit.path;

import toolpathkit.tool.ToolLibrary;
import toolpathkit.setup.Setup;

/** Controller-independent operations and the tools referenced by changes. */
class ToolpathProgram {
  public final ops:Array<ToolpathOp>;
  public final tools:ToolLibrary;
  public final setups:Array<Setup>;

  public function new(ops:Array<ToolpathOp>, tools:ToolLibrary, setups:Array<Setup>) {
    if (ops == null || tools == null || setups == null || setups.length == 0)
      throw "toolpath program needs operations, tools and setups";
    var ids = new Map<String, Bool>();
    for (setup in setups) {
      if (setup == null || ids.exists(setup.id))
        throw "toolpath program has a duplicate or missing setup";
      ids.set(setup.id, true);
    }
    for (op in ops) switch op {
      case SetSetup(id, _):
        if (!ids.exists(id)) throw 'toolpath program names unknown setup $id';
      case _:
    }
    this.ops = ops.copy();
    this.tools = tools;
    this.setups = setups.copy();
  }
}

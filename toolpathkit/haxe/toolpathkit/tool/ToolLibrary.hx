package toolpathkit.tool;

/** Tools referenced by controller-independent tool changes. */
class ToolLibrary {
  final tools:Map<Int, Tool> = new Map();

  public function new() {}

  public function set(tool:Tool):Void {
    if (tool == null) throw "tool must not be null";
    tools.set(tool.number, tool);
  }

  public function tool(number:Int):Tool {
    var result = tools.get(number);
    if (result == null) throw 'Unknown tool $number';
    return result;
  }
}

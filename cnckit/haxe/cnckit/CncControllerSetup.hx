package cnckit;

import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

/** Controller-specific G54–G59, G28/G30, and H/D number mapping. */
class CncControllerSetup {
  final offsets:Map<Int, Array<Float>> = new Map();
  final homes:Map<Int, Array<Float>> = new Map();
  final toolLibrary:ToolLibrary;
  final hTools:Map<Int, Int> = new Map();
  final dTools:Map<Int, Int> = new Map();

  public function new(toolLibrary:ToolLibrary) {
    this.toolLibrary = toolLibrary;
    for (code in 54...60) offsets.set(code, [0.0, 0.0, 0.0]);
    homes.set(28, [0.0, 0.0, 0.0]);
    homes.set(30, [0.0, 0.0, 0.0]);
  }

  public function setupId(code:Int):String {
    if (code < 54 || code > 59) throw 'Unknown CNC work offset G$code';
    return Std.string(code - 53);
  }

  public function setWorkOffset(code:Int, x:Float, y:Float, z:Float):Void {
    setupId(code);
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
      throw "CNC work offset needs finite XYZ";
    offsets.set(code, [x, y, z]);
  }

  public function workOffset(code:Int):Array<Float> {
    setupId(code);
    return offsets.get(code).copy();
  }

  public function setHomePosition(code:Int, x:Float, y:Float, z:Float):Void {
    if ((code != 28 && code != 30) || !Math.isFinite(x) ||
        !Math.isFinite(y) || !Math.isFinite(z))
      throw "CNC home needs G28 or G30 and finite XYZ metres";
    homes.set(code, [x, y, z]);
  }

  public function homePosition(code:Int):Array<Float> {
    var result = homes.get(code);
    if (result == null) throw 'Unknown CNC home G$code';
    return result.copy();
  }

  public function mapH(number:Int, toolId:Int):Void {
    if (number <= 0 || toolId <= 0) throw "H number and tool ID must be positive";
    hTools.set(number, toolId);
  }

  public function mapD(number:Int, toolId:Int):Void {
    if (number <= 0 || toolId <= 0) throw "D number and tool ID must be positive";
    dTools.set(number, toolId);
  }

  public function toolForH(number:Int):Tool
    return toolLibrary.tool(hTools.exists(number) ? hTools.get(number) : number);

  public function toolForD(number:Int):Tool
    return toolLibrary.tool(dTools.exists(number) ? dTools.get(number) : number);

  public function setToolLength(h:Int, length:Float):Void {
    var toolId = hTools.exists(h) ? hTools.get(h) : h;
    var old:Null<Tool> = try toolLibrary.tool(toolId) catch (_:Dynamic) null;
    toolLibrary.set(old == null ? new Tool(toolId, length, 0.0) :
      old.withLength(length));
  }

  public function toolLength(h:Int):Float return toolForH(h).length;
}

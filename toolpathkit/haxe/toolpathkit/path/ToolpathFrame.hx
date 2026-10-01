package toolpathkit.path;

import toolpathkit.setup.Setup;
import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

/**
  The coordinate state in effect at each operation of a program.

  `Move` geometry holds programmed points: work coordinates of the active
  setup, as G-code writes them. The machine's controlled point (the spindle
  gauge line) is the programmed point plus the setup's work origin plus the
  active G43 length in Z. The physical tool tip is that controlled point
  less the loaded tool's own length, so a wrong H number shows up as a
  shifted tip rather than being silently trusted.

  Call `advance` with each operation in order, before using its geometry.
**/
class ToolpathFrame {
  public var setup(default, null):Setup;
  /** The tool number in the spindle; -1 before the first tool change. */
  public var toolNumber(default, null):Int = -1;
  /** The active G43 length; zero after G49 or before any G43. */
  public var toolLength(default, null):Float = 0.0;
  final tools:ToolLibrary;
  final setups:Array<Setup>;

  public function new(tools:ToolLibrary, setups:Array<Setup>) {
    if (tools == null || setups == null || setups.length == 0)
      throw "toolpath frame needs tools and setups";
    this.tools = tools;
    this.setups = setups;
    setup = setups[0];
  }

  public static function of(program:ToolpathProgram):ToolpathFrame
    return new ToolpathFrame(program.tools, program.setups);

  public function advance(op:ToolpathOp):Void {
    switch op {
      case SetSetup(id, _):
        var next:Null<Setup> = null;
        for (candidate in setups) if (candidate.id == id) next = candidate;
        if (next == null) throw 'Unknown setup $id';
        setup = next;
      case ToolChange(number, _):
        toolNumber = number;
      case ToolLengthOffset(_, length, _):
        toolLength = length;
      case _:
    }
  }

  /** Translation from a programmed point to the machine's controlled point. */
  public function toMachine():Array<Float> {
    var origin = setup.workOrigin;
    return [origin.x, origin.y, origin.z + toolLength];
  }

  /** The tool in the spindle, resolved in the program's library. */
  public function tool():Null<Tool>
    return toolNumber < 0 ? null : tools.tool(toolNumber);

  /** Z translation from a programmed point to the physical tool tip. */
  public function tipShift():Float {
    var loaded = tool();
    return toolLength - (loaded == null ? 0.0 : loaded.length);
  }

  /** Translation from a programmed point to the tool tip in work coordinates. */
  public function toTip():Array<Float>
    return [0.0, 0.0, tipShift()];
}

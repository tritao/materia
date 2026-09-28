package stockkit;

import cnckit.CncTool;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;

/** Builds cut moves from CNC operations. */
class CutMoves {
  /**
    Tool-tip moves in the workpiece frame from CNC ops, as produced by CamKit
    or by `CncCompiler.compileDetailed` (cutter compensation already
    resolved). Op geometry is in machine coordinates and includes the active
    G43 tool length, so each point is shifted by `-workOrigin` and by the tool
    length in effect. Tool numbers are resolved with `tools`, for example
    `machine.tool` or `camProgram.tool`. Moves made before any tool change are
    skipped: there is no tool in the spindle to simulate.
  **/
  public static function fromOps(ops:Array<CncOp>, tools:Int->CncTool,
      ?workOrigin:CncPoint):Array<CutMove> {
    var origin = workOrigin == null ? new CncPoint(0, 0, 0) : workOrigin;
    var moves:Array<CutMove> = [];
    var tool:Null<CncTool> = null;
    var toolLength = 0.0;
    for (index in 0...ops.length) switch ops[index] {
      case ToolChange(number, _):
        tool = tools(number);
      case ToolLengthOffset(_, length, _):
        toolLength = length;
      case Rapid(geometry, span):
        if (tool != null) moves.push(new CutMove(tool,
          Path(shift(geometry, origin, toolLength)), true, index, span));
      case Feed(geometry, _, _, span):
        if (tool != null) moves.push(new CutMove(tool,
          Path(shift(geometry, origin, toolLength)), false, index, span));
      case CutterCompStart(_, _, _, _), CutterCompEnd(_):
        throw "cut moves need cutter compensation resolved first";
      case Dwell(_, _), Spindle(_, _, _), Coolant(_, _, _), OptionalStop(_),
          ProgramStop(_), End(_):
    }
    return moves;
  }

  static function shift(geometry:CncGeometry, origin:CncPoint,
      toolLength:Float):CncGeometry {
    function at(point:CncPoint):CncPoint
      return new CncPoint(point.x - origin.x, point.y - origin.y,
        point.z - origin.z - toolLength);
    return switch geometry {
      case Line(start, end): Line(at(start), at(end));
      case Arc(center, radius, startAngle, sweep):
        Arc(at(center), radius, startAngle, sweep);
      case Circular(center, radius, startAngle, sweep, plane, rise):
        Circular(at(center), radius, startAngle, sweep, plane, rise);
    };
  }
}

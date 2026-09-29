package stockkit;

import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathProgram;

/** Builds stock moves from a controller-independent toolpath. */
class CutMoves {
  /**
    Tool-tip moves in the workpiece frame. Op geometry includes the active
    G43 tool length. Each point is shifted by the optional stock origin and
    by the tool length in effect. Tool numbers are resolved with the program library.
    Moves made before any tool change are
    skipped: there is no tool in the spindle to simulate.
  **/
  public static function fromProgram(program:ToolpathProgram,
      ?workOrigin:Point3):Array<CutMove> {
    var ops = program.ops;
    var stockOrigin = workOrigin == null ? new Point3(0, 0, 0) : workOrigin;
    var origin = program.setups[0].workOrigin;
    var moves:Array<CutMove> = [];
    var tool:Null<Tool> = null;
    var toolLength = 0.0;
    for (index in 0...ops.length) switch ops[index] {
      case ToolChange(number, _):
        tool = program.tools.tool(number);
      case ToolLengthOffset(_, length, _):
        toolLength = length;
      case Move(kind, geometry, _, _, provenance):
        if (tool != null) moves.push(new CutMove(tool,
          Path(shift(geometry, origin, stockOrigin, toolLength)), kind, index, provenance));
      case SetSetup(id, _):
        for (setup in program.setups) if (setup.id == id) origin = setup.workOrigin;
      case MachineMove(Rapid, _, _, _, _),
          MachineMove(Link, _, _, _, _),
          MachineMove(Retract, _, _, _, _):
        // Machine travel does not remove stock in the current work setup.
      case MachineMove(_, _, _, _, _):
        throw "stock simulation needs a setup transform for machine-coordinate cuts";
      case Dwell(_, _), Spindle(_, _, _), Coolant(_, _, _), OptionalStop(_),
          ProgramStop(_), End(_):
    }
    return moves;
  }

  static function shift(geometry:PathGeometry, origin:Point3, stockOrigin:Point3,
      toolLength:Float):PathGeometry {
    function at(point:Point3):Point3
      return new Point3(point.x + origin.x - stockOrigin.x,
        point.y + origin.y - stockOrigin.y,
        point.z + origin.z - stockOrigin.z - toolLength);
    return switch geometry {
      case Line(start, end): Line(at(start), at(end));
      case Arc(center, radius, startAngle, sweep):
        Arc(at(center), radius, startAngle, sweep);
      case Circular(center, radius, startAngle, sweep, plane, rise):
        Circular(at(center), radius, startAngle, sweep, plane, rise);
    };
  }
}

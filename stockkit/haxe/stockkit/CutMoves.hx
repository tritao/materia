package stockkit;

import toolpathkit.path.GeometryOffset;
import toolpathkit.path.ToolpathFrame;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathProgram;

/** Builds stock moves from a controller-independent toolpath. */
class CutMoves {
  /**
    Tool-tip moves in the workpiece frame: each programmed point is placed by
    its setup's work origin, the active G43 length and the loaded tool's own
    length (see `ToolpathFrame`), less the optional stock origin. Moves made
    before any tool change are skipped: there is no tool in the spindle to
    simulate.
  **/
  public static function fromProgram(program:ToolpathProgram,
      ?workOrigin:Point3):Array<CutMove> {
    var stockOrigin = workOrigin == null ? new Point3(0, 0, 0) : workOrigin;
    var frame = ToolpathFrame.of(program);
    var moves:Array<CutMove> = [];
    var ops = program.ops;
    for (index in 0...ops.length) {
      var op = ops[index];
      frame.advance(op);
      switch op {
        case Move(kind, geometry, _, _, provenance):
          var tool = frame.tool();
          if (tool == null) continue;
          var origin = frame.setup.workOrigin;
          moves.push(new CutMove(tool, Path(GeometryOffset.translate(geometry,
            [origin.x - stockOrigin.x, origin.y - stockOrigin.y,
              origin.z - stockOrigin.z + frame.tipShift()])),
            kind, index, provenance));
        case MachineMove(Rapid, _, _, _, _),
            MachineMove(Link, _, _, _, _),
            MachineMove(Retract, _, _, _, _):
          // Machine travel does not remove stock in the current work setup.
        case MachineMove(_, _, _, _, _):
          throw "stock simulation needs a setup transform for machine-coordinate cuts";
        case _:
      }
    }
    return moves;
  }
}

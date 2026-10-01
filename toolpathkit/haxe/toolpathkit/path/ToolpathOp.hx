package toolpathkit.path;

import toolpathkit.path.Provenance;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ArcPlane;

/** Controller-independent CNC operations in LinuxCNC block order. */
enum ToolpathOp {
  /** Selects a work-coordinate setup. IDs have no controller-specific meaning. */
  SetSetup(id:String, provenance:Provenance);
  /**
    Programmed points in the active setup's work coordinates, as G-code
    writes them: with G43 active they name the tool tip, and the machine's
    controlled point is the active length above. `ToolpathFrame` maps them.
  **/
  Move(kind:MoveKind, geometry:PathGeometry, feed:Float,
    tolerance:Float, provenance:Provenance);
  /** A move of the controlled point in absolute machine coordinates. */
  MachineMove(kind:MoveKind, geometry:PathGeometry, feed:Float,
    tolerance:Float, provenance:Provenance);
  Dwell(seconds:Float, span:Provenance);
  Spindle(direction:SpindleDirection, rpm:Float, provenance:Provenance);
  Coolant(mist:Bool, flood:Bool, provenance:Provenance);
  ToolChange(number:Int, span:Provenance);
  /**
    G43 Hn sets tool length offset `number` (G49 is number 0). Later `Move`
    points are programmed at the tip, `length` below the controlled point.
  **/
  ToolLengthOffset(number:Int, length:Float, span:Provenance);
  OptionalStop(span:Provenance);
  ProgramStop(span:Provenance);
  End(span:Provenance);
}

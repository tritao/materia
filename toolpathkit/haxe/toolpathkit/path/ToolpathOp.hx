package toolpathkit.path;

import toolpathkit.path.Provenance;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ArcPlane;

/** Controller-independent CNC operations in LinuxCNC block order. */
enum ToolpathOp {
  /** Selects a work-coordinate setup. IDs have no controller-specific meaning. */
  SetSetup(id:String, provenance:Provenance);
  Move(kind:MoveKind, geometry:PathGeometry, feed:Float,
    tolerance:Float, provenance:Provenance);
  /** A move authored in absolute machine coordinates. */
  MachineMove(kind:MoveKind, geometry:PathGeometry, feed:Float,
    tolerance:Float, provenance:Provenance);
  Dwell(seconds:Float, span:Provenance);
  Spindle(direction:SpindleDirection, rpm:Float, provenance:Provenance);
  Coolant(mist:Bool, flood:Bool, provenance:Provenance);
  ToolChange(number:Int, span:Provenance);
  /**
    G43 Hn sets tool length offset `number` (G49 is number 0). Z in later
    geometry already includes `length`, so the tool tip is `length` below it.
  **/
  ToolLengthOffset(number:Int, length:Float, span:Provenance);
  OptionalStop(span:Provenance);
  ProgramStop(span:Provenance);
  End(span:Provenance);
}

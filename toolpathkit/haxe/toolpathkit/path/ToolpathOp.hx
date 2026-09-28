package toolpathkit.path;

import toolpathkit.path.Provenance;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ArcPlane;

/** Controller-independent CNC operations in LinuxCNC block order. */
enum ToolpathOp {
  Move(kind:MoveKind, geometry:PathGeometry, feed:Float,
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
  CutterCompStart(side:Int, radius:Float, plane:ArcPlane, span:Provenance);
  CutterCompEnd(span:Provenance);
  OptionalStop(span:Provenance);
  ProgramStop(span:Provenance);
  End(span:Provenance);
}

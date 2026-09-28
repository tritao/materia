package cnckit.ir;

import cnckit.parse.CncSpan;

/** Controller-independent CNC operations in LinuxCNC block order. */
enum CncOp {
  Rapid(geometry:CncGeometry, span:CncSpan);
  Feed(geometry:CncGeometry, speed:Float, blendTolerance:Float, span:CncSpan);
  Dwell(seconds:Float, span:CncSpan);
  Spindle(channel:String, value:Float, span:CncSpan);
  Coolant(channel:String, enabled:Bool, span:CncSpan);
  ToolChange(number:Int, span:CncSpan);
  /**
    G43 Hn sets tool length offset `number` (G49 is number 0). Z in later
    geometry already includes `length`, so the tool tip is `length` below it.
  **/
  ToolLengthOffset(number:Int, length:Float, span:CncSpan);
  CutterCompStart(side:Int, radius:Float, plane:CncPlane, span:CncSpan);
  CutterCompEnd(span:CncSpan);
  OptionalStop(span:CncSpan);
  ProgramStop(span:CncSpan);
  End(span:CncSpan);
}

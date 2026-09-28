package cnckit.interp;

import toolpathkit.path.ArcPlane;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;

/** Interpreter-only markers resolved before a shared toolpath is returned. */
enum CncInterpOp {
  Path(op:ToolpathOp);
  CutterCompStart(side:Int, radius:Float, plane:ArcPlane,
    provenance:Provenance);
  CutterCompEnd(provenance:Provenance);
}

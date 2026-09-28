package stockkit;

import cnckit.ir.CncGeometry;

/** How the tool moves during one cut, in the workpiece frame. */
enum CutMotion {
  /** The tool tip follows `geometry` with the tool axis along +Z throughout. */
  Path(geometry:CncGeometry);
}

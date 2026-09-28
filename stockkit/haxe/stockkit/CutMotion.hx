package stockkit;

import toolpathkit.path.PathGeometry;

/** How the tool moves during one cut, in the workpiece frame. */
enum CutMotion {
  /** The tool tip follows `geometry` with the tool axis along +Z throughout. */
  Path(geometry:PathGeometry);
}

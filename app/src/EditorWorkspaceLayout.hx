package app;

import nativekit.ui.docking.DockNode;
import nativekit.ui.docking.DockSplitAxis;

/** Default editor dock layout. */
class EditorWorkspaceLayout {
  public static function defaultLayout():DockNode {
    var upper = DockNode.Split(DockSplitAxis.Horizontal, 0.725,
      DockNode.Tabs(["viewport", "perspective"], "viewport"),
      DockNode.Panel("inspector"));
    var main = DockNode.Split(DockSplitAxis.Vertical, 0.76, upper,
      DockNode.Tabs(["console", "telemetry"], "console"));
    return DockNode.Split(DockSplitAxis.Horizontal, 0.20,
      DockNode.Tabs(["hierarchy", "bim", "sensors"], "hierarchy"), main);
  }

}

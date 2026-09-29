package app;

import nativekit.ui.docking.DockNode;
import nativekit.ui.docking.DockSplitAxis;
import nativekit.ui.docking.DockDropZone;
import nativekit.ui.docking.DockWorkspaceModel;

/** Default editor dock layout. */
class EditorWorkspaceLayout {
  /** Replace the retired XY panel while preserving its place in saved layouts. */
  public static function migrateLegacyViewport(workspace:DockWorkspaceModel):Void {
    if (workspace.isOpen("viewport") &&
        (!workspace.isOpen("perspective") || workspace.activePanelId == "viewport"))
      workspace.dock("perspective", "viewport", DockDropZone.Center);
    workspace.unregister("viewport");
  }

  public static function defaultLayout():DockNode {
    var upper = DockNode.Split(DockSplitAxis.Horizontal, 0.725,
      DockNode.Panel("perspective"),
      DockNode.Panel("inspector"));
    var main = DockNode.Split(DockSplitAxis.Vertical, 0.76, upper,
      DockNode.Tabs(["console", "telemetry"], "console"));
    return DockNode.Split(DockSplitAxis.Horizontal, 0.20,
      DockNode.Tabs(["hierarchy", "bim", "sensors"], "hierarchy"), main);
  }

  /** Simulation focus: a wide viewport with telemetry below and runtime controls beside it. */
  public static function simulateLayout():DockNode {
    var main = DockNode.Split(DockSplitAxis.Vertical, 0.68,
      DockNode.Panel("perspective"),
      DockNode.Tabs(["console", "telemetry"], "console"));
    return DockNode.Split(DockSplitAxis.Horizontal, 0.74, main,
      DockNode.Tabs(["sensors", "inspector", "hierarchy"], "sensors"));
  }
}

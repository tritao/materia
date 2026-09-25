package tests;

import app.EditorWorkspaceLayout;
import nativekit.ui.docking.DockNode;
import nativekit.ui.docking.DockSplitAxis;

class EditorWorkspaceLayoutTests {
  public static function main():Int {
    return switch (EditorWorkspaceLayout.defaultLayout()) {
      case DockNode.Split(DockSplitAxis.Horizontal, leftRatio, DockNode.Tabs(left, "hierarchy"),
          DockNode.Split(DockSplitAxis.Vertical, upperRatio,
            DockNode.Split(DockSplitAxis.Horizontal, centerRatio,
              DockNode.Tabs(center, "viewport"), DockNode.Panel("inspector")),
            DockNode.Tabs(bottom, "console"))):
        if (Math.abs(leftRatio - 0.20) > 0.001 ||
            Math.abs((1.0 - leftRatio) * centerRatio - 0.58) > 0.001 ||
            Math.abs((1.0 - leftRatio) * (1.0 - centerRatio) - 0.22) > 0.001 ||
            upperRatio < 0.70 || upperRatio > 0.85 ||
            left.join(",") != "hierarchy,bim,sensors" ||
            center.join(",") != "viewport,perspective" ||
            bottom.join(",") != "console,telemetry") 1 else 0;
      default: 1;
    };
  }
}

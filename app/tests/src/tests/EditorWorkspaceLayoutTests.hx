package tests;

import app.EditorWorkspaceLayout;
import nativekit.ui.docking.DockNode;
import nativekit.ui.docking.DockSplitAxis;
import nativekit.ui.docking.DockPanelDescriptor;
import nativekit.ui.docking.DockWorkspaceModel;

class EditorWorkspaceLayoutTests {
  public static function main():Int {
    var layoutValid = switch (EditorWorkspaceLayout.defaultLayout()) {
      case DockNode.Split(DockSplitAxis.Horizontal, leftRatio, DockNode.Tabs(left, "hierarchy"),
          DockNode.Split(DockSplitAxis.Vertical, upperRatio,
            DockNode.Split(DockSplitAxis.Horizontal, centerRatio,
              DockNode.Panel("perspective"), DockNode.Panel("inspector")),
            DockNode.Tabs(bottom, "console"))):
        if (Math.abs(leftRatio - 0.20) > 0.001 ||
            Math.abs((1.0 - leftRatio) * centerRatio - 0.58) > 0.001 ||
            Math.abs((1.0 - leftRatio) * (1.0 - centerRatio) - 0.22) > 0.001 ||
            upperRatio < 0.70 || upperRatio > 0.85 ||
            left.join(",") != "hierarchy,bim,sensors" ||
            bottom.join(",") != "console,telemetry") 1 else 0;
      default: 1;
    };
    if (layoutValid != 0) return 1;

    for (active in ["viewport", "perspective"]) {
      var workspace = new DockWorkspaceModel();
      for (id in ["viewport", "perspective", "inspector"])
        workspace.register(new DockPanelDescriptor(id, id));
      workspace.setDefaultLayout(DockNode.Split(DockSplitAxis.Horizontal, 0.75,
        DockNode.Tabs(["viewport", "perspective"], active), DockNode.Panel("inspector")));
      workspace.activate(active);
      EditorWorkspaceLayout.migrateLegacyViewport(workspace);
      if (workspace.isOpen("viewport") || !workspace.isOpen("perspective") ||
          workspace.get("viewport") != null || workspace.activePanelId != "perspective") return 1;
    }
    var oldWorkspace = new DockWorkspaceModel();
    for (id in ["viewport", "perspective", "inspector"])
      oldWorkspace.register(new DockPanelDescriptor(id, id));
    oldWorkspace.setDefaultLayout(DockNode.Split(DockSplitAxis.Horizontal, 0.75,
      DockNode.Panel("viewport"), DockNode.Panel("inspector")));
    EditorWorkspaceLayout.migrateLegacyViewport(oldWorkspace);
    return oldWorkspace.isOpen("perspective") && !oldWorkspace.isOpen("viewport") &&
      oldWorkspace.activePanelId == "perspective" ? 0 : 1;
  }
}

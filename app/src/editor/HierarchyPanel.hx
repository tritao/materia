package app.editor;

import app.Main.ReferenceEditorApp;
import Insets;
import LayoutStyle;
import nativekit.ui.core.View;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.collections.TreeView;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.ButtonVariant;
import nativekit.ui.widgets.controls.SearchField;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.Row;

/** Hierarchy dock panel. */
@:access(app.Main.ReferenceEditorApp)
class HierarchyPanel {
  public static function build(app:ReferenceEditorApp):View {
    var addStyle = new LayoutStyle();
    addStyle.padding = new Insets(6.0, 8.0, 6.0, 8.0);
    addStyle.childGap = 5.0;
    var addButton = new Button("Add", addStyle, function() {
      app.hierarchyAddVisible = true;
      app.invalidateView();
    }, "hierarchy-add");
    addButton.variant = ButtonVariant.Secondary;
    addButton.leadingIcon = IconName.Plus;
    addButton.trailingIcon = IconName.ChevronDown;
    addButton.onClickEvent = function(event) {
      var bounds = app.menuTriggerBounds(event);
      app.hierarchyAddX = Math.max(8.0, Math.min(app.viewportWidth - 228.0, bounds.x));
      app.hierarchyAddY = Math.max(8.0, Math.min(app.viewportHeight - 560.0, bounds.y + bounds.height));
      app.hierarchyAddVisible = true;
      app.invalidateView();
    };
    var treeStyle = ReferenceEditorApp.fillStyle();
    treeStyle.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    treeStyle.background = app.appearance.theme.tokens.surface;
    treeStyle.childGap = 6.0;
    var treeViewport = ReferenceEditorApp.fillStyle();
    var tree = new TreeView(app.hierarchySearch == "" ? "scene-hierarchy" : "scene-hierarchy-filtered",
      app.treeModel, treeViewport, null, 420.0, app.scene.treeSelectionKey(), ["scene"], function(id) {
      app.scene.selectTreeKey(id);
      app.log("Selected " + id);
      app.invalidateView();
    }, function(id) {
      app.scene.selectTreeKey(id);
      app.commands.execute("scene.frame-selected");
      app.log("Framed " + id);
    }, null, null);
    tree.onExpandedChanged = function(_, _) app.hierarchyExpansionRevision++;
    tree.onItemContextMenu = function(id, event) {
      if (app.scene.object(id) == null) return;
      app.hierarchyMenuX = Math.max(8.0, Math.min(app.viewportWidth - 228.0, event.x));
      app.hierarchyMenuY = Math.max(8.0, Math.min(app.viewportHeight - 180.0, event.y));
      app.hierarchyMenuVisible = true;
      app.invalidateView();
    };
    tree.onItemRename = app.startRename;
    return new Column(
      "hierarchy-panel",
      [
        new KeyedView("heading", app.sectionHeading("SCENE")),
        new KeyedView("actions", new Row("scene-object-actions", [
          new KeyedView("add", addButton),
          new KeyedView("duplicate", app.sceneAction("scene-duplicate", "scene.duplicate", "", IconName.Copy)),
          new KeyedView("delete", app.sceneAction("scene-delete", "scene.delete", "", IconName.Trash))
        ], ReferenceEditorApp.actionRowStyle())),
        new KeyedView("search", new SearchField("hierarchy-search", app.hierarchySearch, function(value) {
          app.hierarchySearch = value;
          app.treeModel.setFilter(value);
          app.invalidateView();
        }, null, "Search objects...")),
        new KeyedView(
          "tree",
          tree
        )
      ],
      treeStyle
    );
  }

}

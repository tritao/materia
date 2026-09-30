package tests;

import app.AppSettings;
import app.Main.ReferenceEditorApp;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;
import nativekit.ui.properties.PropertyValue;
import FontCollection;
import LayoutFrame;
import nativekit.ui.widgets.settings.SettingsPanel;
import sys.FileSystem;

/** The Editor Settings dialog: opening, editing, resetting and closing it in a laid-out editor. */
class EditorSettingsDialogTests {
  static final DIRECTORY = "build/editor-settings-dialog-test";

  public static function main():Int {
    try { run(); Sys.println("Editor settings dialog tests passed"); return 0; }
    catch (error:Dynamic) { Sys.println('Editor settings dialog tests failed: $error'); return 1; }
  }

  static function check(condition:Bool, message:String):Void
    if (!condition) throw message;

  static function run():Void {
    if (!FileSystem.exists("build")) FileSystem.createDirectory("build");
    if (!FileSystem.exists(DIRECTORY)) FileSystem.createDirectory(DIRECTORY);
    var settingsFile = DIRECTORY + "/settings.json";
    if (FileSystem.exists(settingsFile)) FileSystem.deleteFile(settingsFile);
    var fontPath:Null<String> = null;
    for (candidate in ["uikit/vendor/harfbuzz/perf/fonts/Roboto-Regular.ttf",
      "../../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"])
      if (FileSystem.exists(candidate)) fontPath = candidate;
    check(fontPath != null, "a test font is available");
    var fonts = FontCollection.create();
    fonts.add(cast fontPath);
    var editor = new ReferenceEditorApp(fonts, DIRECTORY + "/workspace.json");
    var frame = new LayoutFrame(1320.0, 900.0);
    editor.submit(frame);
    check(editor.settingsPanel == null && find(editor.submit(frame), "editor-settings-close") == null,
      "the dialog starts closed");

    check(editor.commands.dispatch(UiKey.Comma, UiModifier.Control), "Ctrl+, is bound");
    check(editor.settingsPanel != null, "Ctrl+, opens Editor Settings");
    var root = editor.submit(frame);
    check(visible(root, "editor-settings-close"), "the dialog is laid out with its Close button");
    var header:RenderNode = cast find(root, "editor-settings:header");
    check(header != null && visible(header, "input"), "the filter is shown");
    check(visible(root, "editor-settings:advanced"), "the Advanced switch is shown");
    check(panel(editor).selectedCategory == "interface/start_page", "the first category is selected");

    // Edit a setting through its row: the store, the preferences and the file all follow.
    var toggle = "editor:" + AppSettings.SHOW_START_PAGE;
    check(visible(root, toggle), "the Start page setting has a row");
    check(find(root, "reset") == null, "no reset arrow while every value is at its default");
    click(editor, frame, toggle);
    check(!editor.preferences.showStartPage, "clicking the row changes the setting");
    check(FileSystem.exists(settingsFile), "the change is saved");
    check(visible(editor.submit(frame), "reset"), "a changed value offers the reset arrow");
    click(editor, frame, "reset");
    check(editor.preferences.showStartPage && editor.preferences.store.isDefault(AppSettings.SHOW_START_PAGE),
      "the reset arrow restores the default");
    check(find(editor.submit(frame), "reset") == null, "and then disappears");

    // A setting changed in the dialog reaches the running editor.
    panel(editor).select("editors");
    check(panel(editor).selectedCategory == "editors/3d", "the 3D category can be selected");
    check(visible(editor.submit(frame), "editor:" + AppSettings.ANTIALIASING + ":numeric"), "the anti-aliasing row is shown");
    editor.preferences.store.set(AppSettings.ANTIALIASING, PropertyValue.Int(2));
    check(editor.antialiasing == 2, "the viewport follows the anti-aliasing setting");
    editor.setAntialiasing(8);
    check(editor.preferences.antialiasing == 8, "the viewport's own control still saves the setting");

    // Filtering narrows the tree; a filter with no match says so.
    panel(editor).setFilter("anti");
    check(panel(editor).categories().join(",") == "editors/3d", "the filter narrows the categories");
    panel(editor).setFilter("zzz");
    check(panel(editor).selectedCategory == null, "nothing matches");
    editor.submit(frame);

    click(editor, frame, "editor-settings-close");
    check(editor.settingsPanel == null && find(editor.submit(frame), "editor-settings-close") == null,
      "Close dismisses the dialog");
    editor.dispose();
    fonts.dispose();
  }

  static function click(editor:ReferenceEditorApp, frame:LayoutFrame, key:String):Void {
    var node:RenderNode = cast find(editor.submit(frame), key);
    check(node != null && node.resolved != null, "a control exists: " + key);
    var bounds = node.resolved.clippedViewportBounds();
    check(bounds.width > 0 && bounds.height > 0, "a control is visible: " + key);
    var x = bounds.x + bounds.width / 2, y = bounds.y + bounds.height / 2;
    editor.ui.pointerDown(x, y, 0);
    editor.ui.pointerUp(x, y, 0);
    editor.submit(frame);
  }

  static function visible(root:RenderNode, key:String):Bool {
    var node:RenderNode = cast find(root, key);
    if (node == null || node.resolved == null) return false;
    var bounds = node.resolved.clippedViewportBounds();
    return bounds.width > 0 && bounds.height > 0;
  }

  static function find(node:RenderNode, key:String):Null<RenderNode> {
    if (node.styleKey == key) return node;
    for (child in node.children) {
      var found = find(child, key);
      if (found != null) return found;
    }
    return null;
  }

  static function panel(editor:ReferenceEditorApp):SettingsPanel {
    var open:Null<SettingsPanel> = editor.settingsPanel;
    if (open == null) throw "the settings dialog is not open";
    return open;
  }

}

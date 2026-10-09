package tests;

import app.AppSettings;
import app.Main.ReferenceEditorApp;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import haxeon.ui.core.UiModifier;
import haxeon.ui.properties.PropertyValue;
import haxeon.ui.theme.Theme;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.widgets.settings.SettingsPanel;
import haxeon.ui.widgets.settings.ShortcutsPanel;
import sys.FileSystem;

/** The Editor Settings dialog: opening, editing, resetting and closing it in a laid-out editor. */
@:access(app.Main.ReferenceEditorApp)
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
    var fontPath = TestPaths.font();
    check(FileSystem.exists(fontPath), "a test font is available");
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

    // Shortcuts tab: rebind Editor Settings to Ctrl+Shift+O by clicking its shortcut and pressing the chord.
    editor.showSettingsTab("shortcuts");
    root = editor.submit(frame);
    check(!visible(root, "editor:" + AppSettings.ANTIALIASING + ":numeric"), "the General tab is hidden");
    check(find(root, "shortcut:editor.undo") != null && find(root, "shortcut:editor.settings") != null,
      "the Shortcuts tab lists the commands");
    shortcuts(editor).setFilter("editor settings");
    root = editor.submit(frame);
    check(visible(root, "shortcut:editor.settings") && find(root, "shortcut:editor.undo") == null,
      "the filter narrows the list");
    click(editor, frame, "shortcut:editor.settings");
    check(shortcuts(editor).recordingId == "editor.settings", "clicking a shortcut records");
    editor.ui.key(UiEventKind.KeyDown, 340, UiModifier.Shift);
    editor.ui.key(UiEventKind.KeyDown, 79, UiModifier.Control | UiModifier.Shift);
    editor.submit(frame);
    check(shortcuts(editor).recordingId == null, "the chord ends recording");
    check(shortcuts(editor).shortcutText("editor.settings") == "Ctrl+Shift+O",
      "the chord is assigned: " + shortcuts(editor).shortcutText("editor.settings"));
    check(editor.settingsPanel != null, "the recorded chord did not run anything or close the dialog");
    click(editor, frame, "shortcut:editor.settings");
    editor.ui.key(UiEventKind.KeyDown, UiKey.Escape, 0);
    editor.submit(frame);
    check(editor.settingsPanel != null && shortcuts(editor).shortcutText("editor.settings") == "Ctrl+Shift+O",
      "Escape cancels recording without closing the dialog");
    check(visible(editor.submit(frame), "shortcut-reset:editor.settings"), "a changed shortcut offers reset");

    click(editor, frame, "editor-settings-close");
    check(editor.settingsPanel == null && find(editor.submit(frame), "editor-settings-close") == null,
      "Close dismisses the dialog");
    check(!editor.commands.dispatch(UiKey.Comma, UiModifier.Control), "the old chord no longer opens settings");
    check(editor.commands.dispatch(79, UiModifier.Control | UiModifier.Shift) && editor.settingsPanel != null,
      "the new chord does");
    editor.closeSettings();
    editor.dispose();

    viewSettings(fonts);

    // A restarted editor keeps the new binding.
    var restarted = new ReferenceEditorApp(fonts, DIRECTORY + "/workspace.json");
    check(!restarted.commands.dispatch(UiKey.Comma, UiModifier.Control)
      && restarted.commands.dispatch(79, UiModifier.Control | UiModifier.Shift) && restarted.settingsPanel != null,
      "custom shortcuts survive a restart");
    restarted.shortcutBindings.reset("editor.settings");
    check(restarted.commands.shortcutsFor("editor.settings")[0].label() == "Ctrl+,", "reset restores Ctrl+,");
    restarted.dispose();
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

  static function shortcuts(editor:ReferenceEditorApp):ShortcutsPanel {
    var open:Null<ShortcutsPanel> = editor.shortcutsPanel;
    if (open == null) throw "the shortcuts tab is not open";
    return open;
  }

  static function panel(editor:ReferenceEditorApp):SettingsPanel {
    var open:Null<SettingsPanel> = editor.settingsPanel;
    if (open == null) throw "the settings dialog is not open";
    return open;
  }


  /** The grid, lighting and theme: menu commands save them, the dialog changes them, a restart keeps them. */
  static function viewSettings(fonts:FontCollection):Void {
    var editor = new ReferenceEditorApp(fonts, DIRECTORY + "/workspace.json");
    var store = editor.preferences.store;
    check(editor.gridVisible && !editor.gridSnapEnabled && editor.gridSpacing == 0.2 && editor.lightingPreset == 0
      && !editor.appearance.dark, "view settings start at their defaults");

    check(editor.commands.execute("scene.toggle-grid") && !editor.gridVisible && !store.getBool(AppSettings.GRID_VISIBLE),
      "Toggle grid saves the choice");
    check(editor.commands.get("scene.toggle-grid").isChecked() == false, "and the menu shows it");
    check(editor.simulationOverlaysVisible && editor.commands.execute("scene.toggle-simulation-overlays") &&
      !editor.simulationOverlaysVisible && !store.getBool(AppSettings.SIMULATION_OVERLAYS) &&
      editor.commands.get("scene.toggle-simulation-overlays").isChecked() == false,
      "Toggle simulation overlays saves the choice and the menu shows it");
    editor.commands.execute("scene.toggle-simulation-overlays");
    editor.commands.execute("scene.toggle-grid-snap");
    editor.commands.execute("scene.grid-spacing-0.5");
    check(editor.gridSnapEnabled && editor.gridSpacing == 0.5 && store.getFloat(AppSettings.GRID_SPACING) == 0.5,
      "snapping and spacing commands save their choices");
    check(editor.commands.get("scene.grid-spacing-0.5").isChecked(), "the spacing menu item is checked");

    store.set(AppSettings.GRID_SPACING, PropertyValue.Float(0.25));
    check(editor.gridSpacing == 0.25 && !editor.commands.get("scene.grid-spacing-0.5").isChecked(),
      "a spacing typed in the dialog reaches the editor");
    store.set(AppSettings.LIGHTING, PropertyValue.Enum("contrast"));
    check(editor.lightingPreset == 2 && editor.commands.get("scene.lighting-contrast").isChecked(),
      "lighting chosen in the dialog reaches the editor and its menu");

    check(editor.commands.execute("editor.toggle-dark-theme") && editor.appearance.dark
      && store.getString(AppSettings.COLOR_SCHEME) == "dark", "Toggle dark theme saves the color scheme");
    store.set(AppSettings.COLOR_SCHEME, PropertyValue.Enum("light"));
    check(!editor.appearance.dark, "the color scheme chosen in the dialog applies at once");
    store.set(AppSettings.COLOR_SCHEME, PropertyValue.Enum("dark"));
    editor.dispose();

    var restarted = new ReferenceEditorApp(fonts, DIRECTORY + "/workspace.json");
    check(restarted.appearance.dark && !restarted.gridVisible && restarted.gridSnapEnabled && restarted.gridSpacing == 0.25
      && restarted.lightingPreset == 2, "a restarted editor starts with the saved view settings");
    var launchedLight = new ReferenceEditorApp(fonts, DIRECTORY + "/workspace.json", Theme.light());
    check(!launchedLight.appearance.dark && launchedLight.preferences.store.getString(AppSettings.COLOR_SCHEME) == "dark",
      "a theme given at launch wins for that run without changing the saved one");
    launchedLight.commands.execute("editor.toggle-dark-theme");
    check(launchedLight.appearance.dark, "toggling from a launch theme still switches");
    launchedLight.dispose();
    for (path in [AppSettings.GRID_VISIBLE, AppSettings.GRID_SNAP, AppSettings.GRID_SPACING, AppSettings.LIGHTING,
      AppSettings.COLOR_SCHEME])
      restarted.preferences.store.reset(path);
    restarted.dispose();
  }
}

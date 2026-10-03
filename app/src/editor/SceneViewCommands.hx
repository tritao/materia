package app.editor;

import app.AppSettings;
import app.Main.ReferenceEditorApp;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.core.Command;
import nativekit.ui.core.Shortcut;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;

/**
 * Viewport, lighting, grid, and nudge command registrations. Grid and lighting
 * commands change the saved settings; the editor applies them from there.
 */
@:access(app.Main.ReferenceEditorApp)
class SceneViewCommands {
  public static function install(app:ReferenceEditorApp):Void {
    app.commands.register(new Command("scene.frame-selected", "Frame selected", function() {
      if (app.perspectiveViewport != null) app.perspectiveViewport.frameSelected();
      app.log("Framed " + app.scene.selectedId);
    }, null, function() return !app.documents.blocked() && app.scene.items().length > 0));
    app.commands.register(new Command("scene.reset-perspective", "Reset perspective view", function() {
      if (app.perspectiveViewport != null) app.perspectiveViewport.resetView();
      app.log("Perspective view reset");
    }, null, function() return !app.documents.blocked() && app.perspectiveViewport != null));
    registerLightingPreset(app, "scene.lighting-studio", "Lighting: Studio", 0);
    registerLightingPreset(app, "scene.lighting-soft", "Lighting: Soft", 1);
    registerLightingPreset(app, "scene.lighting-contrast", "Lighting: Contrast", 2);
    registerAntialiasing(app, "scene.antialiasing-off", "Anti-aliasing: Off", 1);
    registerAntialiasing(app, "scene.antialiasing-2x", "Anti-aliasing: 2x", 2);
    registerAntialiasing(app, "scene.antialiasing-4x", "Anti-aliasing: 4x", 4);
    app.commands.register(new Command("scene.toggle-grid", "Toggle grid", function() {
      app.preferences.store.set(AppSettings.GRID_VISIBLE, PropertyValue.Bool(!app.gridVisible));
      app.log(app.gridVisible ? "Grid enabled" : "Grid disabled");
    }, null, null, function() return app.gridVisible));
    app.commands.register(new Command("scene.toggle-simulation-overlays", "Toggle simulation overlays", function() {
      app.preferences.store.set(AppSettings.SIMULATION_OVERLAYS, PropertyValue.Bool(!app.simulationOverlaysVisible));
      app.log(app.simulationOverlaysVisible ? "Simulation overlays shown" : "Simulation overlays hidden");
    }, null, null, function() return app.simulationOverlaysVisible));
    app.commands.register(new Command("scene.toggle-grid-snap", "Toggle grid snapping", function() {
      app.preferences.store.set(AppSettings.GRID_SNAP, PropertyValue.Bool(!app.gridSnapEnabled));
      app.log(app.gridSnapEnabled ? "Grid snapping enabled" : "Grid snapping disabled");
    }, null, null, function() return app.gridSnapEnabled));
    registerGridSpacing(app, "scene.grid-spacing-0.1", "Grid spacing: 0.1 m", 0.1);
    registerGridSpacing(app, "scene.grid-spacing-0.2", "Grid spacing: 0.2 m", 0.2);
    registerGridSpacing(app, "scene.grid-spacing-0.5", "Grid spacing: 0.5 m", 0.5);
    registerNudgeCommand(app, "scene.nudge-left", "Nudge left", UiKey.Left, 0, -0.1, 0.0);
    registerNudgeCommand(app, "scene.nudge-right", "Nudge right", UiKey.Right, 0, 0.1, 0.0);
    registerNudgeCommand(app, "scene.nudge-up", "Nudge up", UiKey.Up, 0, 0.0, 0.1);
    registerNudgeCommand(app, "scene.nudge-down", "Nudge down", UiKey.Down, 0, 0.0, -0.1);
    registerNudgeCommand(app, "scene.nudge-left-large", "Nudge left (large)", UiKey.Left,
      UiModifier.Shift, -1.0, 0.0);
    registerNudgeCommand(app, "scene.nudge-right-large", "Nudge right (large)", UiKey.Right,
      UiModifier.Shift, 1.0, 0.0);
    registerNudgeCommand(app, "scene.nudge-up-large", "Nudge up (large)", UiKey.Up,
      UiModifier.Shift, 0.0, 1.0);
    registerNudgeCommand(app, "scene.nudge-down-large", "Nudge down (large)", UiKey.Down,
      UiModifier.Shift, 0.0, -1.0);
    app.commands.register(new Command("scene.cancel-drag", "Cancel object drag", function() {
      app.cancelActiveDrag();
      app.commands.refresh();
    }, new Shortcut(UiKey.Escape), function() return
      (app.perspectiveViewport != null && app.perspectiveViewport.dragging()) || app.scene.hasActiveSketchEdit()));
  }

  static function registerGridSpacing(app:ReferenceEditorApp, id:String, label:String, spacing:Float):Void {
    app.commands.register(new Command(id, label, function() {
      app.preferences.store.set(AppSettings.GRID_SPACING, PropertyValue.Float(spacing));
      app.log("Grid spacing set to " + spacing + " m");
    }, null, null, function() return app.gridSpacing == spacing));
  }

  static function registerLightingPreset(app:ReferenceEditorApp, id:String, label:String, preset:Int):Void {
    app.commands.register(new Command(id, label, function() {
      app.preferences.store.set(AppSettings.LIGHTING, PropertyValue.Enum(AppSettings.LIGHTING_PRESETS[preset]));
      app.log(label);
    }, null, function() return !app.documents.blocked() && app.perspectiveViewport != null,
      function() return app.lightingPreset == preset));
  }

  static function registerAntialiasing(app:ReferenceEditorApp, id:String, label:String, samples:Int):Void {
    app.commands.register(new Command(id, label, function() {
      app.setAntialiasing(samples);
      app.log(label);
      app.commands.refresh();
    }, null, function() return !app.documents.blocked() && app.perspectiveViewport != null &&
      app.perspectiveViewport.supportsSamples(samples),
      function() return app.antialiasing == samples));
  }

  static function registerNudgeCommand(app:ReferenceEditorApp, id:String, label:String, key:Int, modifiers:Int,
      deltaX:Float, deltaY:Float):Void {
    app.commands.register(new Command(id, label, function() {
      app.scene.nudgeSelected(deltaX, deltaY);
      app.commands.refresh();
    }, new Shortcut(key, modifiers), function() return app.canEditObjects() &&
      app.scene.object(app.scene.selectedId) != null));
  }

}

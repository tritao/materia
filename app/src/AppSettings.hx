package app;

import app.editor.EditorGrid;
import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.settings.SettingOptions;
import nativekit.ui.settings.SettingsRegistry;

/** Every editor setting Materia defines, by path. */
class AppSettings {
  public static inline var SHOW_START_PAGE:String = "interface/start_page/show_at_startup";
  /** Samples per pixel for the 3D viewport; one means anti-aliasing is off. */
  public static inline var ANTIALIASING:String = "editors/3d/antialiasing_samples";
  /** "light" or "dark". */
  public static inline var COLOR_SCHEME:String = "interface/theme/color_scheme";
  /** One of LIGHTING_PRESETS; its index is the viewport's preset number. */
  public static inline var LIGHTING:String = "editors/3d/lighting";
  public static inline var GRID_VISIBLE:String = "editors/3d/grid/show_grid";
  public static inline var GRID_SNAP:String = "editors/3d/grid/snap_to_grid";
  /** Metres between grid lines, and the snapping step. */
  public static inline var GRID_SPACING:String = "editors/3d/grid/spacing";

  public static final LIGHTING_PRESETS:Array<String> = ["studio", "soft", "contrast"];

  /** A registry holding every setting the editor defines. */
  public static function registry():SettingsRegistry {
    var registry = new SettingsRegistry();
    define(registry);
    return registry;
  }

  public static function define(registry:SettingsRegistry):Void {
    var startPage = new SettingOptions();
    startPage.tooltip = "Open the Start page when Materia starts without a file to open.";
    registry.define(SHOW_START_PAGE, PropertyType.Bool, PropertyValue.Bool(true), startPage);

    var antialiasing = new SettingOptions();
    antialiasing.label = "Anti-Aliasing Samples";
    antialiasing.tooltip = "Samples per pixel for edges in the 3D viewport; 1 turns anti-aliasing off.";
    antialiasing.minimum = 1;
    antialiasing.maximum = 16;
    antialiasing.step = 1;
    registry.define(ANTIALIASING, PropertyType.Int, PropertyValue.Int(AppPreferences.DEFAULT_ANTIALIASING), antialiasing);

    var scheme = new SettingOptions();
    scheme.options = [new PropertyOption("light", "Light"), new PropertyOption("dark", "Dark")];
    registry.define(COLOR_SCHEME, PropertyType.Enum, PropertyValue.Enum("light"), scheme);

    var lighting = new SettingOptions();
    lighting.tooltip = "How the 3D viewport lights the scene.";
    lighting.options = [new PropertyOption("studio", "Studio"), new PropertyOption("soft", "Soft"),
      new PropertyOption("contrast", "Contrast")];
    registry.define(LIGHTING, PropertyType.Enum, PropertyValue.Enum("studio"), lighting);

    registry.define(GRID_VISIBLE, PropertyType.Bool, PropertyValue.Bool(true));
    var snap = new SettingOptions();
    snap.tooltip = "Round dragged and placed objects to the grid spacing.";
    registry.define(GRID_SNAP, PropertyType.Bool, PropertyValue.Bool(false), snap);
    var spacing = new SettingOptions();
    spacing.minimum = 0.01;
    spacing.maximum = 10.0;
    spacing.step = 0.01;
    spacing.unit = "m";
    registry.define(GRID_SPACING, PropertyType.Float, PropertyValue.Float(EditorGrid.STEP), spacing);
  }
}

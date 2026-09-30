package app;

import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.settings.SettingOptions;
import nativekit.ui.settings.SettingsRegistry;

/** Every editor setting Materia defines, by path. */
class AppSettings {
  public static inline var SHOW_START_PAGE:String = "interface/start_page/show_at_startup";
  /** Samples per pixel for the 3D viewport; one means anti-aliasing is off. */
  public static inline var ANTIALIASING:String = "editors/3d/antialiasing_samples";

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
  }
}

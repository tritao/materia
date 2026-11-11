package app;

import app.editor.EditorGrid;
import haxeon.ui.properties.PropertyOption;
import haxeon.ui.properties.PropertyType;
import haxeon.ui.properties.PropertyValue;
import haxeon.ui.settings.SettingOptions;
import haxeon.ui.settings.SettingsRegistry;

/** Every editor setting Materia defines, by path. */
class AppSettings {
  public static inline var SHOW_START_PAGE:String = "interface/start_page/show_at_startup";
  /** Samples per pixel for the 3D viewport; one means anti-aliasing is off. */
  public static inline var ANTIALIASING:String = "editors/3d/antialiasing_samples";
  /** "light" or "dark". */
  public static inline var COLOR_SCHEME:String = "interface/theme/color_scheme";
  /** One of LIGHTING_PRESETS; its index is the viewport's preset number. */
  public static inline var LIGHTING:String = "editors/3d/lighting";
  public static inline var COSMETIC_THREADS:String = "editors/3d/show_cosmetic_threads";
  /** Draw CAD edges of opaque parts from the visible surfaces instead of as strokes (app/plans/SCREEN_EDGES.md). */
  public static inline var SCREEN_EDGES:String = "editors/3d/screen_space_edges";
  public static inline var GRID_VISIBLE:String = "editors/3d/grid/show_grid";
  public static inline var GRID_SNAP:String = "editors/3d/grid/snap_to_grid";
  /** Metres between grid lines, and the snapping step. */
  public static inline var GRID_SPACING:String = "editors/3d/grid/spacing";

  public static inline var COLLISION_DETAILS:String = "editors/3d/annotations/collision_details";
  public static inline var SENSOR_MOUNTS:String = "editors/3d/annotations/sensors";
  public static inline var SENSOR_READINGS:String = "editors/3d/annotations/readings";
  public static inline var LINK_FRAMES:String = "editors/3d/annotations/link_frames";
  public static inline var ANNOTATIONS_THROUGH_GEOMETRY:String = "editors/3d/annotations/through_geometry";
  /** Master visibility of viewport inspection aids and live mission overlays. */
  public static inline var SIMULATION_OVERLAYS:String = "editors/3d/simulation/show_overlays";
  /** Whether the viewport tints and joins objects that collide or come near while editing (COLLISION.md CL8b). */
  public static inline var COLLISIONS_VISIBLE:String = "editors/3d/collision/show";
  /** How close two objects count as near, in millimetres. */
  public static inline var COLLISION_NEAR:String = "editors/3d/collision/near_mm";

  /** Where a Klipper printer's Moonraker listens, `host` or `host:port`; empty until one is set. */
  public static inline var PRINTER_ENDPOINT:String = "machine/printer/moonraker_endpoint";
  /** The Klipper `printer.cfg` that Install Klipper config sends (the printer example exports one); empty until one is set. */
  public static inline var PRINTER_CONFIG:String = "machine/printer/klipper_config_file";

  public static final LIGHTING_PRESETS:Array<String> = haxeon.editor.ViewportLook.PresetNames;

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
    registry.define(
      ANTIALIASING,
      PropertyType.Int,
      PropertyValue.Int(AppPreferences.DEFAULT_ANTIALIASING),
      antialiasing
    );

    var scheme = new SettingOptions();
    scheme.options = [new PropertyOption("light", "Light"), new PropertyOption("dark", "Dark")];
    registry.define(COLOR_SCHEME, PropertyType.Enum, PropertyValue.Enum("light"), scheme);

    var lighting = new SettingOptions();
    lighting.tooltip = "How the 3D viewport lights the scene.";
    lighting.options = [
      new PropertyOption("studio", "Studio"),
      new PropertyOption("soft", "Soft"),
      new PropertyOption("contrast", "Contrast")
    ];
    registry.define(LIGHTING, PropertyType.Enum, PropertyValue.Enum("studio"), lighting);

    var printer = new SettingOptions();
    printer.label = "Moonraker Address";
    printer.tooltip = "The host (and port, 7125 unless given) of the Klipper printer that Send to printer talks to.";
    registry.define(PRINTER_ENDPOINT, PropertyType.Text, PropertyValue.Text(""), printer);
    var config = new SettingOptions();
    config.label = "Klipper Config File";
    config.tooltip =
      "The printer.cfg that Install Klipper config uploads to the printer, restarts Klipper on, and reads back.";
    registry.define(PRINTER_CONFIG, PropertyType.Text, PropertyValue.Text(""), config);

    registry.define(GRID_VISIBLE, PropertyType.Bool, PropertyValue.Bool(true));
    var threads = new SettingOptions();
    threads.label = "Show Cosmetic Threads";
    threads.tooltip = "Draw the nominal threads on named bore and shank faces.";
    registry.define(COSMETIC_THREADS, PropertyType.Bool, PropertyValue.Bool(true), threads);
    var edges = new SettingOptions();
    edges.label = "Screen-Space Edges";
    edges.tooltip = "Draw the edges of opaque parts from what is visible, so hidden edges never show through thin walls.";
    registry.define(SCREEN_EDGES, PropertyType.Bool, PropertyValue.Bool(false), edges);
    var overlays = new SettingOptions();
    overlays.label = "Show Viewport Overlays";
    overlays.tooltip =
      "Show enabled inspection annotations and live mission overlays.";
    registry.define(SIMULATION_OVERLAYS, PropertyType.Bool, PropertyValue.Bool(true), overlays);
    for (entry in [
      {path: COLLISION_DETAILS, label: "Collision details", tooltip: "Show closest-point markers for all collision and proximity pairs; selection reveals related pairs automatically."},
      {path: SENSOR_MOUNTS, label: "Sensors", tooltip: "Show all sensor mounts; the inspected sensor is shown automatically."},
      {path: SENSOR_READINGS, label: "Sensor readings", tooltip: "Show available sensor readings in the viewport."},
      {path: LINK_FRAMES, label: "Link frames", tooltip: "Show robot link origins and orientation axes."},
      {path: ANNOTATIONS_THROUGH_GEOMETRY, label: "Show annotations through geometry",
        tooltip: "Reveal inspection aids hidden behind machine geometry."}
    ]) {
      var option = new SettingOptions();
      option.label = entry.label;
      option.tooltip = entry.tooltip;
      registry.define(entry.path, PropertyType.Bool, PropertyValue.Bool(false), option);
    }
    var collisions = new SettingOptions();
    collisions.label = "Show Collisions";
    collisions.tooltip =
      "While editing, tint objects that collide red and objects nearer than the near distance amber, and join their closest points.";
    registry.define(COLLISIONS_VISIBLE, PropertyType.Bool, PropertyValue.Bool(true), collisions);
    var near = new SettingOptions();
    near.label = "Near Distance";
    near.tooltip = "Objects closer than this are shown as near.";
    near.minimum = 0.0;
    near.maximum = 1000.0;
    near.step = 1.0;
    near.unit = "mm";
    registry.define(COLLISION_NEAR, PropertyType.Float, PropertyValue.Float(10.0), near);
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

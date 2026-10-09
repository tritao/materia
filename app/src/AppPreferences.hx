package app;

import haxeon.ui.Path;

import haxe.Json;
import haxe.io.Path as PreferencesPath;
import haxeon.ui.properties.PropertyValue;
import haxeon.ui.settings.SettingsJsonValue;
import haxeon.ui.settings.SettingsStore;
import sys.FileSystem;
import sys.io.File;

/**
 * Per-user settings kept beside the workspace layout, plus the recent-files list.
 *
 * The values live in a SettingsStore (see AppSettings for the definitions), so the
 * settings dialog edits the same values. A preferences.json written by earlier builds
 * is imported once, when no settings file exists yet, and left in place for them.
 */
class AppPreferences {
  public static inline var MAX_RECENT:Int = 10;
  /** Samples per pixel for the 3D viewport's edges on a GPU that supports it. */
  public static inline var DEFAULT_ANTIALIASING:Int = haxeon.editor.ViewportLook.SampleCount;
  static inline var RECENT_STATE:String = "recent";

  public final store:SettingsStore;
  public var showStartPage(get, never):Bool;
  /** Samples per pixel for the 3D viewport; one means anti-aliasing is off. */
  public var antialiasing(get, never):Int;
  public final recent:Array<String> = [];

  public function new(file:String, ?legacyFile:String) {
    var imported = !FileSystem.exists(file) && legacyFile != null && FileSystem.exists(legacyFile);
    store = new SettingsStore(AppSettings.registry(), file);
    if (imported) importLegacy(legacyFile);
    readRecent(store.getState(RECENT_STATE), recent);
  }

  public static function besideWorkspace(workspacePath:String):AppPreferences {
    var directory = PreferencesPath.directory(workspacePath);
    return new AppPreferences(PreferencesPath.join([directory, "settings.json"]),
      PreferencesPath.join([directory, "preferences.json"]));
  }

  function get_showStartPage():Bool
    return store.getBool(AppSettings.SHOW_START_PAGE);

  function get_antialiasing():Int
    return store.getInt(AppSettings.ANTIALIASING);

  public function setShowStartPage(value:Bool):Void
    store.set(AppSettings.SHOW_START_PAGE, PropertyValue.Bool(value));

  /** Counts outside one to sixteen are refused and leave the saved choice alone. */
  public function setAntialiasing(value:Int):Void
    store.set(AppSettings.ANTIALIASING, PropertyValue.Int(value));

  /** Moves the path to the front of the recent list, dropping duplicates and the oldest entries. */
  public function addRecent(path:String):Void {
    if (path == null || path.length == 0) return;
    var absolute = FileSystem.exists(path) ? FileSystem.fullPath(path) : path;
    recent.remove(absolute);
    recent.unshift(absolute);
    while (recent.length > MAX_RECENT) recent.pop();
    store.setState(RECENT_STATE, recent.copy());
  }

  public function forgetRecent(path:String):Void {
    if (recent.remove(path)) store.setState(RECENT_STATE, recent.copy());
  }

  /** Copies the paths that are strings, up to the recent-list limit. */
  static function readRecent(stored:Dynamic, into:Array<String>):Void {
    if (!SettingsJsonValue.isArray(stored)) return;
    for (path in (stored:Array<Dynamic>))
      if (SettingsJsonValue.isString(path) && into.length < MAX_RECENT) into.push(path);
  }

  /** Copies the values earlier builds kept in preferences.json; unusable values are skipped. */
  function importLegacy(legacyFile:String):Void {
    var data:Dynamic = null;
    try data = Json.parse(File.getContent(legacyFile)) catch (_:Dynamic) return;
    if (data == null || !Reflect.isObject(data)) return;
    store.batch(function() {
      if (Reflect.field(data, "showStartPage") == false) setShowStartPage(false);
      var storedAntialiasing:Dynamic = Reflect.field(data, "antialiasing");
      if (SettingsJsonValue.isInt(storedAntialiasing)) setAntialiasing(storedAntialiasing);
      var paths:Array<String> = [];
      readRecent(Reflect.field(data, "recent"), paths);
      if (paths.length > 0) store.setState(RECENT_STATE, paths);
    });
  }
}

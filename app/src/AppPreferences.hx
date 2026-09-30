package app;

import haxe.Json;
import haxe.io.Path as PreferencesPath;
import sys.FileSystem;
import sys.io.File;

/** Small per-user settings kept beside the workspace layout: the Start page toggle, anti-aliasing and recent files. */
class AppPreferences {
  public static inline var MAX_RECENT:Int = 10;
  /** Samples per pixel for the 3D viewport's edges on a GPU that supports it. */
  public static inline var DEFAULT_ANTIALIASING:Int = 4;

  final file:String;
  public var showStartPage(default, null):Bool = true;
  /** Samples per pixel for the 3D viewport; one means anti-aliasing is off. */
  public var antialiasing(default, null):Int = DEFAULT_ANTIALIASING;
  public final recent:Array<String> = [];

  public function new(file:String) {
    this.file = file;
    load();
  }

  public static function besideWorkspace(workspacePath:String):AppPreferences
    return new AppPreferences(PreferencesPath.join([PreferencesPath.directory(workspacePath), "preferences.json"]));

  public function setShowStartPage(value:Bool):Void {
    if (showStartPage == value) return;
    showStartPage = value;
    save();
  }

  public function setAntialiasing(value:Int):Void {
    if (value < 1 || value > 16 || antialiasing == value) return;
    antialiasing = value;
    save();
  }

  /** Moves the path to the front of the recent list, dropping duplicates and the oldest entries. */
  public function addRecent(path:String):Void {
    if (path == null || path.length == 0) return;
    var absolute = FileSystem.exists(path) ? FileSystem.fullPath(path) : path;
    recent.remove(absolute);
    recent.unshift(absolute);
    while (recent.length > MAX_RECENT) recent.pop();
    save();
  }

  public function forgetRecent(path:String):Void {
    if (recent.remove(path)) save();
  }

  function load():Void {
    try {
      if (!FileSystem.exists(file)) return;
      var data:Dynamic = Json.parse(File.getContent(file));
      if (Reflect.field(data, "showStartPage") == false) showStartPage = false;
      var storedAntialiasing:Dynamic = Reflect.field(data, "antialiasing");
      if (Std.isOfType(storedAntialiasing, Int) && storedAntialiasing >= 1 && storedAntialiasing <= 16)
        antialiasing = storedAntialiasing;
      var stored:Dynamic = Reflect.field(data, "recent");
      if (Std.isOfType(stored, Array))
        for (path in (stored:Array<Dynamic>))
          if (Std.isOfType(path, String) && recent.length < MAX_RECENT) recent.push(path);
    } catch (_:Dynamic) {
      // A missing or damaged preferences file only resets these conveniences.
    }
  }

  function save():Void {
    try {
      var directory = PreferencesPath.directory(file);
      if (directory.length > 0 && !FileSystem.exists(directory)) FileSystem.createDirectory(directory);
      File.saveContent(file, Json.stringify({showStartPage: showStartPage, antialiasing: antialiasing, recent: recent}));
    } catch (_:Dynamic) {
      // Preferences are best effort; failing to write them must not disturb editing.
    }
  }
}

package machinekit.catalog;

import haxe.io.Path;
import sys.FileSystem;

/**
 * Where the repository is, for data that lives in the checkout (`build/suppliers`) and must be found from any working directory:
 * a program runs with its project's directory as its working directory (the editor runs a generated artifact in its example
 * directory), which is not the repository root. `MATERIA_ROOT` wins. Otherwise the root is the nearest directory above the working
 * directory, else above the program, that holds `machinekit/haxe/src`.
 */
class RepositoryRoot {
  /** A directory is the repository root when it holds this one. */
  public static final marker = "machinekit/haxe/src";

  static var found:Null<String>;
  static var searched = false;

  /** The root, or null when `MATERIA_ROOT` is unset and no directory above the working directory or the program holds the marker. */
  public static function find():Null<String> {
    var configured = Sys.getEnv("MATERIA_ROOT");
    if (configured != null && configured != "") return configured;
    if (!searched) {
      searched = true;
      found = search(Sys.getCwd());
      if (found == null) found = search(Path.directory(Sys.programPath()));
    }
    return found;
  }

  /** `relative` inside the repository, else `relative` itself (found from the working directory as before). */
  public static function path(relative:String):String {
    var root = find();
    return root == null ? relative : root + "/" + relative;
  }

  /** The nearest directory at or above `start` that holds the marker, else null. */
  public static function search(start:String):Null<String> {
    if (start == null || start == "") return null;
    var directory = Path.normalize(start);
    while (directory.length > 1 && StringTools.endsWith(directory, "/")) directory = directory.substr(
      0,
      directory.length - 1
    );
    while (true) {
      if (directory != "" && FileSystem.exists(directory + "/" + marker)
        && FileSystem.isDirectory(directory + "/" + marker)) return directory;
      var parent = Path.directory(directory);
      if (parent == directory || parent == "") return null;
      directory = parent;
    }
  }

  /** For tests: forget the cached search. */
  public static function reset():Void {
    found = null;
    searched = false;
  }
}

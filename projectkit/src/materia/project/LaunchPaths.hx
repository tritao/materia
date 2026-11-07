package materia.project;

import haxe.io.Path;
import sys.FileSystem;

/**
 * Paths given on a program's command line mean the directory the user started it from. `haxeon run` runs a program with its
 * project's directory as the working directory and tells it where the user was in `HAXEON_INVOKED_FROM`; a program started any
 * other way has no such variable, and its working directory is where it was started.
 */
class LaunchPaths {
  public static final variable = "HAXEON_INVOKED_FROM";

  /** The directory the user started the program from. */
  public static function invokedFrom():String {
    var value = Sys.getEnv(variable);
    return value != null && value != "" ? value : Sys.getCwd();
  }

  /** `path` as an absolute, normalised path, taking a relative one from the start directory. A path that does not exist still resolves. */
  public static function resolve(path:String):String {
    if (path == null || path == "") return "";
    var absolute = Path.isAbsolute(path) ? path : Path.join([invokedFrom(), path]);
    var result = Path.normalize(absolute);
    while (result.length > 1 && StringTools.endsWith(result, "/")) result = result.substr(0, result.length - 1);
    return result;
  }

  /** The real path of `path` when it exists, else what `resolve` gives: never null, so a message about a missing file can name it. */
  public static function existing(path:String):String {
    var resolved = resolve(path);
    if (resolved == "" || !FileSystem.exists(resolved)) return resolved;
    var real = FileSystem.fullPath(resolved);
    return real == null ? resolved : real;
  }
}

package tests;

import sys.FileSystem;

/**
 * Paths for the app tests from the repository root, wherever the suite was
 * started inside it: the nearest directory holding `app/src/Main.hx` at or
 * above the working directory. No test changes the working directory to find
 * its files.
 */
class TestPaths {
  static var root:Null<String> = null;

  /** The font the editor tests lay text out with. */
  public static function font():String
    return of("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf");

  /** `relative` (from the repository root) as a path that opens from any working directory. */
  public static function of(relative:String):String
    return repository() + "/" + relative;

  public static function repository():String {
    if (root != null) return root;
    var found = above(FileSystem.fullPath(Sys.getCwd()));
    if (found == null) throw "The app tests run inside the repository (no app/src/Main.hx above the working directory)";
    return root = found;
  }

  static function above(directory:String):Null<String> {
    var current = directory;
    while (StringTools.endsWith(current, "/")) current = current.substr(0, current.length - 1);
    while (current.length > 0) {
      if (FileSystem.exists(current + "/app/src/Main.hx")) return current;
      var parent = haxe.io.Path.directory(current);
      if (parent == current) break;
      current = parent;
    }
    return null;
  }
}

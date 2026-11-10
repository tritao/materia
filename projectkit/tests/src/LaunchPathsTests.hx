import materia.project.LaunchPaths;
import sys.FileSystem;
import sys.io.File;

/** Command-line paths mean the directory the user started from, not the directory `haxeon run` gives the program. */
class LaunchPathsTests {
  static var assertions = 0;

  static function same(actual:String, expected:String, message:String):Void {
    assertions++;
    if (actual != expected) throw message + ": " + actual + " vs " + expected;
  }

  public static function run():Int {
    assertions = 0;
    var saved = Sys.getEnv(LaunchPaths.variable);
    var home = Sys.getCwd();
    try {
      Sys.putEnv(LaunchPaths.variable, "/work/repo");
      same(LaunchPaths.invokedFrom(), "/work/repo", "the start directory comes from HAXEON_INVOKED_FROM");
      same(
        LaunchPaths.resolve("machinekit/examples/hobber/materia.project.json"),
        "/work/repo/machinekit/examples/hobber/materia.project.json",
        "a relative path is taken from the start directory"
      );
      same(LaunchPaths.resolve("./a/../b/project.json"), "/work/repo/b/project.json", "dots are folded");
      same(LaunchPaths.resolve("/abs/project.json"), "/abs/project.json", "an absolute path is left alone");
      same(LaunchPaths.resolve("dir/"), "/work/repo/dir", "a trailing slash is dropped");
      same(LaunchPaths.resolve(""), "", "an empty path stays empty");
      same(LaunchPaths.resolve(null), "", "a null path becomes empty, never null");

      // Unset or empty: the working directory is the start directory.
      Sys.putEnv(LaunchPaths.variable, "");
      same(LaunchPaths.invokedFrom(), Sys.getCwd(), "an empty variable means the working directory");
      same(
        LaunchPaths.resolve("x.json"),
        LaunchPaths.resolve(Sys.getCwd() + "/x.json"),
        "so relative paths are taken from it"
      );

      // A path that does not exist still resolves, so the message can name it (it used to print "null").
      Sys.putEnv(LaunchPaths.variable, "/work/repo");
      var missing = LaunchPaths.existing("machinekit/examples/flexspline-hobber/materia.project.json");
      same(
        missing,
        "/work/repo/machinekit/examples/flexspline-hobber/materia.project.json",
        "a missing project file is named by its resolved path"
      );
      assertions++;
      if (missing == null) throw "existing never returns null";

      // A path that exists is returned as its real path, found from the start directory rather than the working directory.
      var tmp = Sys.getEnv("TMPDIR");
      if (tmp == null || tmp == "") tmp = "/tmp";
      var start = tmp + "/materia-launchpaths-test-" + Sys.getPid();
      var elsewhere = start + "/project-dir";
      FileSystem.createDirectory(start);
      FileSystem.createDirectory(elsewhere);
      File.saveContent(start + "/p.json", "{}");
      Sys.putEnv(LaunchPaths.variable, start);
      Sys.setCwd(elsewhere);
      same(
        LaunchPaths.existing("p.json"),
        FileSystem.fullPath(start + "/p.json"),
        "an existing file is found from the start directory, not the working directory"
      );
      assertions++;
      if (FileSystem.exists("p.json")) throw "the working directory must not hold the file for this to prove anything";
    } catch (error:Dynamic) {
      Sys.setCwd(home);
      Sys.putEnv(LaunchPaths.variable, saved == null ? "" : saved);
      throw error;
    }
    Sys.setCwd(home);
    Sys.putEnv(LaunchPaths.variable, saved == null ? "" : saved);
    return assertions;
  }
}

import machinekit.catalog.RepositoryRoot;
import machinekit.catalog.SupplierLibraries;
import sys.FileSystem;

/** Finding the repository root (and the supplier libraries in it) from any working directory. */
class RepositoryRootTests {
  static var passed = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    passed++;
  }

  static function same(a:Null<String>, b:Null<String>, message:String):Void check(
    a == b,
    message + ": " + a + " vs " + b
  );

  static function directories(path:String):Void {
    var parent = haxe.io.Path.directory(path);
    if (parent != path && parent != "" && !FileSystem.exists(parent)) directories(parent);
    if (!FileSystem.exists(path)) FileSystem.createDirectory(path);
  }

  public static function main():Void {
    var tmp = Sys.getEnv("TMPDIR");
    if (tmp == null || tmp == "") tmp = "/tmp";
    var base = tmp + "/materia-root-test-" + Sys.getPid();
    var fake = base + "/fake";
    var deep = fake + "/machinekit/examples/hobber/build/run";
    var outside = base + "/outside/a/b";
    directories(fake + "/" + RepositoryRoot.marker);
    directories(deep);
    directories(outside);
    var home = Sys.getCwd();
    var savedRoot = Sys.getEnv("MATERIA_ROOT");
    try {
      // search: nearest directory at or above the start that holds the marker.
      same(RepositoryRoot.search(deep), fake, "found from deep inside the repository");
      same(RepositoryRoot.search(fake), fake, "found at the root itself");
      same(RepositoryRoot.search(deep + "/"), fake, "a trailing slash does not matter");
      same(RepositoryRoot.search(outside), null, "nothing found outside any repository");
      same(RepositoryRoot.search(""), null, "an empty start finds nothing");
      same(RepositoryRoot.search(null), null, "a null start finds nothing");
      // a nearer marker wins over a farther one
      directories(deep + "/inner/" + RepositoryRoot.marker);
      same(RepositoryRoot.search(deep + "/inner"), deep + "/inner", "the nearest marker wins");

      // find: MATERIA_ROOT wins, else the working directory, else the program.
      Sys.putEnv("MATERIA_ROOT", "/somewhere/explicit");
      RepositoryRoot.reset();
      same(RepositoryRoot.find(), "/somewhere/explicit", "MATERIA_ROOT wins");
      same(RepositoryRoot.path("build/suppliers/x"), "/somewhere/explicit/build/suppliers/x", "path joins it");
      Sys.putEnv("MATERIA_ROOT", "");
      Sys.setCwd(deep);
      RepositoryRoot.reset();
      same(RepositoryRoot.find(), fake, "found from the working directory when MATERIA_ROOT is unset");
      same(RepositoryRoot.path("build/suppliers/x"), fake + "/build/suppliers/x", "path is inside the found root");
      // the result is cached: moving away does not change it until reset
      Sys.setCwd(outside);
      same(RepositoryRoot.find(), fake, "the search result is kept");
      RepositoryRoot.reset();
      var fromProgram = RepositoryRoot.find();
      check(
        fromProgram == null || FileSystem.exists(fromProgram + "/" + RepositoryRoot.marker),
        "outside any repository it falls back to the program's location, and what it finds is a root"
      );

      // The supplier libraries follow it, and their own overrides still win.
      Sys.setCwd(deep);
      RepositoryRoot.reset();
      same(
        SupplierLibraries.hzmotion(),
        fake + "/build/suppliers/hzmotion",
        "hzmotion library from deep inside the repository"
      );
      same(SupplierLibraries.yuheng(), fake + "/build/suppliers/yuheng", "yuheng library too");
      Sys.putEnv("MATERIA_HZMOTION_ROOT", "/data/hz");
      same(SupplierLibraries.hzmotion(), "/data/hz", "the library's own variable wins");
      Sys.putEnv("MATERIA_HZMOTION_ROOT", "");
      Sys.putEnv("MATERIA_ZPP_ROOT", "");
      same(
        machinekit.milling.ZppComponent.directory().indexOf("/build/suppliers/zppcnc") > 0 && StringTools.startsWith(
          machinekit.milling.ZppComponent.directory(),
          fake
        ) ? "ok" : machinekit.milling.ZppComponent.directory(),
        "ok",
        "zpp library inside the found root"
      );
      Sys.putEnv("MATERIA_HG_REFERENCE", "");
      same(
        machinekit.motion.LinearRailBlock.referenceDirectory(),
        fake + "/build/suppliers/vallder/hg-reference",
        "HG reference inside the found root"
      );
    } catch (error:Dynamic) {
      Sys.setCwd(home);
      Sys.putEnv("MATERIA_ROOT", savedRoot == null ? "" : savedRoot);
      throw error;
    }
    Sys.setCwd(home);
    Sys.putEnv("MATERIA_ROOT", savedRoot == null ? "" : savedRoot);
    Sys.println("repository root tests: " + passed + " checks passed");
  }
}

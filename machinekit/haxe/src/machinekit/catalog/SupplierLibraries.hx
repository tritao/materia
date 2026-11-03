package machinekit.catalog;

/**
 * Where each supplier's prepared CAD library lives: deployment configuration, never saved in recipes. Each library
 * holds hash-addressed files under `cad/assets/<sha256>.step`.
 */
class SupplierLibraries {
  static var vallderOverride:Null<String>;

  /** For tests and tools that prepare the library elsewhere. */
  public static function setVallderDirectory(path:String):Void {
    if (path == null || path.length == 0) throw "Vallder library directory is empty";
    vallderOverride = path;
  }

  /** `MATERIA_VALLDER_ROOT`, else build/suppliers/vallder in the repository. */
  public static function vallder():String {
    if (vallderOverride != null) return vallderOverride;
    return configured("MATERIA_VALLDER_ROOT", "build/suppliers/vallder");
  }

  /** `MATERIA_HZMOTION_ROOT`, else build/suppliers/hzmotion in the repository. */
  public static function hzmotion():String return configured("MATERIA_HZMOTION_ROOT", "build/suppliers/hzmotion");

  /** `MATERIA_ZEROERR_ROOT`, else build/suppliers/zeroerr in the repository. */
  public static function zeroerr():String return configured("MATERIA_ZEROERR_ROOT", "build/suppliers/zeroerr");

  /** The pinned file `sha256` in `library`. */
  public static function asset(library:String, sha256:String):String return library + "/cad/assets/" + sha256 + ".step";

  static function configured(variable:String, relative:String):String {
    var value = Sys.getEnv(variable);
    if (value != null && value.length > 0) return value;
    var root = Sys.getEnv("MATERIA_ROOT");
    return root == null ? relative : root + "/" + relative;
  }
}

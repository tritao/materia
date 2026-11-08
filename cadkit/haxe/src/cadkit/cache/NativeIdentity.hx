package cadkit.cache;

import cadkit.Shape;

/**
 * The native build a stored CadKit result came from, as one text for the key of every persistent cache: the loaded CadKit library's
 * content hash (`CodeIdentity.loadedLibrariesContent`), the OCCT version text plus the loaded OCCT libraries (names and sizes), the
 * naming-scheme version and `MATERIA_CACHE_SALT` (set it after changing something these cannot see). Null when the loaded CadKit
 * library cannot be identified, which turns the persistent tiers off, with a line on stderr.
 */
class NativeIdentity {
  static var resolved = false;
  static var value:Null<String>;

  public static function get():Null<String> {
    if (!resolved) {
      resolved = true;
      value = compute();
    }
    return value;
  }

  static function compute():Null<String> {
    // The first CadKit call loads the native library and with it OCCT's, so ask for the version before listing what is loaded.
    var version = Shape.occtVersion(), salt = Sys.getEnv("MATERIA_CACHE_SALT");
    // The library the process loaded, wherever it was found; the directory beside the program only where the loader cannot say.
    var cadkit = haxeon.build.CodeIdentity.loadedLibrariesContent("libcadkit") ?? haxeon.build.CodeIdentity.nativeBuild("cadkit");
    if (cadkit == null) {
      Sys.stderr().writeString("kernel cache off: no native identity (the loaded CadKit library could not be identified)\n");
      return null;
    }
    var libraries = haxeon.build.CodeIdentity.loadedLibraries("libTK");
    return "cadkit:" + cadkit + "|occt:" + version + (libraries
      == null ? "" : "+" + libraries) + (salt == null ? "" : "|salt:" + salt) + "|naming:" + Shape.namingScheme();
  }
}

package cadkit.cache;

import cadkit.Shape;

/**
 * The native build a stored CadKit result came from, as one text for the key of every persistent cache: the CadKit library's
 * content hash (`CodeIdentity.nativeBuild`), the OCCT version text plus the loaded OCCT libraries (names and sizes), the
 * naming-scheme version and `MATERIA_MEMO_SALT` (set it after changing something these cannot see). Null when the program has
 * no CadKit build beside it, which turns the persistent tiers off.
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
    var cadkit = haxeon.build.CodeIdentity.nativeBuild("cadkit");
    if (cadkit == null) return null;
    // The first CadKit call loads the native library and with it OCCT's, so ask for the version before listing what is loaded.
    var version = Shape.occtVersion(), salt = Sys.getEnv("MATERIA_MEMO_SALT");
    var libraries = haxeon.build.CodeIdentity.loadedLibraries("libTK");
    return "cadkit:" + cadkit + "|occt:" + version + (libraries
      == null ? "" : "+" + libraries) + (salt == null ? "" : "|salt:" + salt) + "|naming:" + Shape.namingScheme();
  }
}

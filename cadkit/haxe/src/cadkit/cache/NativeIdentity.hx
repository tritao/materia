package cadkit.cache;

import cadkit.Shape;
import haxe.io.Path;
import haxeon.build.CodeIdentity;
import sys.FileSystem;
import sys.io.File;

/**
 * The native build a stored CadKit result came from, as one text for the key of every persistent cache: the content hash of the
 * loaded CadKit library, the identity stamp of the OCCT install the loaded OCCT libraries came from (`occt.identity` beside its
 * `lib` directory, written by `cadkit/scripts/build-occt-prefix.sh`) and the naming-scheme version. Null when either cannot be
 * found, which turns the persistent tiers off, with a line on stderr that names what is missing.
 */
class NativeIdentity {
  /** The file in an OCCT install that identifies its build. */
  public static inline var OCCT_STAMP = "occt.identity";

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
    // The first CadKit call loads the native library and with it OCCT's, so make it before asking what is loaded.
    Shape.occtVersion();
    var cadkit = CodeIdentity.loadedLibrariesContent("libcadkit");
    if (cadkit == null) return off("the loaded CadKit library could not be identified");
    var stamps:Array<String> = [];
    for (library in CodeIdentity.mappedLibraries("libTK")) {
      var stamp = Path.join([Path.directory(Path.directory(library)), OCCT_STAMP]);
      if (stamps.indexOf(stamp) >= 0) continue;
      if (!FileSystem.exists(stamp)) return off("the OCCT install has no " + OCCT_STAMP + " (" + stamp + ")");
      stamps.push(stamp);
    }
    if (stamps.length == 0) return off("no OCCT library is loaded");
    stamps.sort(Reflect.compare);
    return "cadkit:" + cadkit + "|occt:" + [for (stamp in stamps) StringTools.trim(File.getContent(stamp))].join("+") + "|naming:"
      + Shape.namingScheme();
  }

  static function off(reason:String):Null<String> {
    Sys.stderr().writeString("kernel cache off: no native identity (" + reason + ")\n");
    return null;
  }
}

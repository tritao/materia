package tests;

import sys.FileSystem;

/** Finds checked-in files whether tests run from the repository root or a project directory. */
class Files {
  /** Path of a file relative to robotkit/, e.g. "policy/tests/fixtures/affine_state.onnx". */
  public static function robotkit(relative:String):String {
    var cwd = Sys.getCwd();
    for (prefix in ["/robotkit/", "/", "/../", "/../../", "/../../../", "/../../../../"]) {
      var candidate = cwd + prefix + relative;
      if (FileSystem.exists(candidate)) return candidate;
    }
    throw 'cannot find robotkit/$relative from $cwd';
  }
}

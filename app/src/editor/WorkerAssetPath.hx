package app.editor;

import sys.FileSystem;

/** Resolves a document-relative bundled asset from app and test working directories. */
class WorkerAssetPath {
  public static function resolve(path:String):String {
    if (FileSystem.exists(path)) return path;
    for (prefix in ["../", "../../", "../../../", "../../../../"])
      if (FileSystem.exists(prefix + path)) return prefix + path;
    return path;
  }
}

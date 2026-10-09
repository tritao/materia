package app;

import haxe.io.Bytes;
import sys.FileSystem;
import sys.io.File;
import sys.io.AtomicFile;

/** Desktop filesystem or browser virtual filesystem mirrored to OPFS by BrowserFiles. */
class FilePreparedSceneStore implements PreparedSceneStore {
  final directory:String;
  public function new(directory:String) this.directory = directory;
  function path(key:String):String {
    if (key.length != 64) throw "Invalid prepared cache key";
    for (index in 0...key.length) {
      var code = key.charCodeAt(index);
      if (!(code >= 48 && code <= 57 || code >= 97 && code <= 102)) throw "Invalid prepared cache key";
    }
    return directory + "/" + key + ".mtrp";
  }
  public function read(key:String):Null<Bytes> {
    var file = path(key);
    if (!FileSystem.exists(file)) return null;
    var metadata = FileSystem.metadata(file);
    if (metadata == null || metadata.size > 150000000) throw "Prepared cache is too large";
    return File.getBytes(file);
  }
  public function write(key:String, bytes:Bytes):Void {
    FileSystem.createDirectory(directory);
    AtomicFile.writeBytes(path(key), bytes);
  }
}

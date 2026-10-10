package app;

import haxe.io.Bytes;

/** Disposable portable preparation blobs, addressed by a content-derived key. */
interface PreparedSceneStore {
  public function read(key:String):Null<Bytes>;
  public function write(key:String, bytes:Bytes):Void;
}

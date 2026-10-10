package app;

import haxe.crypto.Sha256;
import haxe.io.Bytes;

/** Disposable derived data. Never stores native handles or replaces the source artifact. */
class PreparedProjectCache {
  static inline var VERSION:String = "materia-prepared-scene/2:centered:float32:hull-thickness=0.0005m";
  final context:Null<PreparedSceneCaches.PreparedSceneCacheContext>;
  final key:String;

  public function new(artifactHash:String, ?context:PreparedSceneCaches.PreparedSceneCacheContext) {
    this.context = context;
    key = context == null ? "" : hex(Sha256.make(Bytes.ofString(VERSION + ":" + context.buildIdentity + ":" + artifactHash)));
  }

  public function read(parts:Array<materia.project.SceneArtifact.SceneArtifactPart>):Null<Array<PreparedProjectPart>> {
    if (context == null) return null;
    try {
      var bytes = context.store.read(key);
      if (bytes == null) return null;
      return PreparedSceneCodec.decode(bytes, parts);
    } catch (error:Dynamic) {
      Sys.println("Materia project: rebuilding prepared cache: " + Std.string(error));
      // Corruption is a cache miss; a successful preparation atomically replaces it.
      return null;
    }
  }

  public function write(parts:Array<PreparedProjectPart>):Void {
    if (context == null) return;
    try {
      context.store.write(key, PreparedSceneCodec.encode(parts));
    } catch (_:Dynamic) {}
  }

  public static function hex(bytes:Bytes):String {
    var digits = "0123456789abcdef", result = new StringBuf();
    for (i in 0...bytes.length) { var value = bytes.get(i); result.add(digits.charAt(value >>> 4)); result.add(digits.charAt(value & 15)); }
    return result.toString();
  }
}

import haxe.io.Bytes;
import haxe.crypto.Sha256;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import sys.FileSystem;
import sys.io.File;

/** Cache only the fixed, source-generated reference scenes, never custom cells or collision answers.
 * Keys include the actual bytecode, executable and every mapped native library's contents.
 * A new process recomputes this identity; missing platform support disables caching.
 */
class TendingGeometryCache {
  final directory:Null<String>;
  public function new(enabled:Bool) {
    if (enabled) {
      Sys.println("Reach geometry: checking loaded-code cache identity");
      directory = runtimeDirectory();
    } else directory = null;
  }

  static function runtimeDirectory():Null<String> {
    if (Sys.systemName() != "Linux" || Sys.args().indexOf("--no-geometry-cache") >= 0) return null;
    try {
      var program = Sys.programPath();
      if (!StringTools.endsWith(program, ".hl") || !FileSystem.exists(program)) throw 'No stable bytecode file: $program';
      var paths = [FileSystem.fullPath(program), FileSystem.fullPath(Sys.executablePath())];
      // Some supported runtimes treat procfs's reported zero size as an empty
      // file. Resolve "self" in this process before the streaming reader starts.
      var mapsPath = FileSystem.fullPath("/proc/self/maps");
      if (mapsPath.indexOf("/proc/self/") >= 0) throw "Cannot resolve process identity";
      var reader = sys.io.Process.run("cat", [mapsPath]);
      var maps = reader.readStdout();
      var readStatus = reader.exitCode(); reader.close();
      if (readStatus != 0) throw "Cannot read loaded native libraries";
      for (line in maps.split("\n")) {
        var start = line.indexOf("/");
        if (start < 0) continue;
        var path = line.substr(start);
        if (path.indexOf(".so") < 0 && path.indexOf(".hdll") < 0 && line.indexOf(" r-x") < 0) continue;
        if (!FileSystem.exists(path)) throw 'Mapped library is unavailable: $path';
        if (paths.indexOf(path) < 0) paths.push(path);
      }
      if (paths.length <= 2) throw "Cannot discover loaded native libraries";
      paths.sort((a,b) -> a < b ? -1 : a > b ? 1 : 0);
      // Native hashing avoids copying hundreds of megabytes through the VM.
      var key = contentKey(paths);
      var root = Sys.getEnv("XDG_CACHE_HOME");
      if (root == null || root.length == 0) {
        var home = Sys.getEnv("HOME");
        if (home == null || home.length == 0) return null;
        root = home + "/.cache";
      }
      return root + "/materia/tending-geometry-v1/" + key;
    } catch (error:Dynamic) {
      Sys.println('Reach geometry: disk cache unavailable ($error)');
      return null;
    }
  }

  static function contentKey(paths:Array<String>):String {
    var hash = sys.io.Process.run("sha256sum", paths);
    var identity = hash.readStdout();
    var status = hash.exitCode(); hash.close();
    if (status != 0 || identity.split("\n").length != paths.length + 1) throw "Cannot hash loaded code";
    return Sha256.encode(identity);
  }

  static function ensureDirectory(path:String):Void {
    if (FileSystem.exists(path)) return;
    var parent = haxe.io.Path.directory(path);
    if (parent.length > 0 && parent != path) ensureDirectory(parent);
    FileSystem.createDirectory(path);
    if (!FileSystem.exists(path)) throw "Cannot create geometry cache directory";
  }

  public function scene(name:String, build:Void -> SceneArtifactData):{data:SceneArtifactData, bytes:Bytes} {
    var path = directory == null ? null : directory + "/" + name + ".scene";
    return artifact(path, build);
  }

  static function artifact(path:Null<String>, build:Void -> SceneArtifactData):{data:SceneArtifactData, bytes:Bytes} {
    if (path != null && FileSystem.exists(path)) {
      try {
        var packed = File.getBytes(path);
        if (packed.length < 32 || packed.length > SceneArtifact.MAX_BYTES + 32) throw "Invalid cached scene length";
        var bytes = packed.sub(32, packed.length - 32), digest = Sha256.make(bytes);
        for (i in 0...32) if (packed.get(i) != digest.get(i)) throw "Cached scene digest mismatch";
        var data = SceneArtifact.decode(bytes);
        Sys.println('Reach geometry: reused ${haxe.io.Path.withoutDirectory(path)}');
        return {data:data, bytes:bytes};
      } catch (_:Dynamic) {
        // Corrupt/truncated cache entries are disposable; regenerate from source.
      }
    }
    var data = build(), bytes = SceneArtifact.encode(data);
    if (path != null) {
      try {
        ensureDirectory(haxe.io.Path.directory(path));
        var packed = Bytes.alloc(32 + bytes.length), digest = Sha256.make(bytes);
        packed.blit(0, digest, 0, 32); packed.blit(32, bytes, 0, bytes.length);
        // A partially written entry is detected by its digest by any other reader.
        File.saveBytes(path, packed);
      } catch (_:Dynamic) {
        // Read-only/full cache storage must not prevent validation.
      }
    }
    return {data:data, bytes:bytes};
  }
}

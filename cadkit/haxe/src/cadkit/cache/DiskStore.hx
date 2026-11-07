package cadkit.cache;

import haxe.crypto.Sha256;
import haxe.io.Bytes;
import sys.FileSystem;
import sys.io.AtomicFile;
import sys.io.File;

/**
 * The persistent tier behind `Memo.persist`: one file per entry in a directory, named by the SHA-256 of the full key, with a
 * size cap and least-recently-used eviction (by file modification time, which a hit refreshes).
 *
 * Writes are atomic (temp file then rename), so concurrent writers and readers, in this or another process, see either the
 * whole entry or none. Two writers of one key write the same bytes, since the key names everything the value depends on.
 * A failed read or write is a miss, never an error: the cache only ever makes a run faster.
 *
 * Environment: `MATERIA_MEMO_DIR` (directory; default `$XDG_CACHE_HOME/materia/memo`, else `~/.cache/materia/memo`),
 * `MATERIA_MEMO=off` (no persistent tier), `MATERIA_MEMO_MAX_MB` (cap, default 2048), `MATERIA_MEMO_VERIFY=1` (see `Memo`).
 */
class DiskStore {
  static final defaultCapMegabytes = 2048;
  /** A hit older than this is rewritten to refresh its recency; younger hits cost nothing. */
  static final refreshSeconds = 3600;

  public final directory:String;
  public final capBytes:Float;
  var written:Float;
  var checked:Bool;

  public function new(directory:String, capBytes:Float) {
    this.directory = directory;
    this.capBytes = capBytes;
    written = 0;
    checked = false;
  }

  static var shared:Null<DiskStore>;
  static var sharedResolved = false;
  static var verifyOverride:Null<Bool>;

  /** For tests: forces verify mode on or off (null: follow the environment). */
  public static function forceVerify(on:Null<Bool>):Void verifyOverride = on;

  /** The process-wide store from the environment, or null when the persistent tier is off. */
  public static function process():Null<DiskStore> {
    if (!sharedResolved) {
      sharedResolved = true;
      shared = fromEnvironment();
    }
    return shared;
  }

  /** For tests: replaces (or with null, switches off) the process-wide store. */
  public static function useProcess(store:Null<DiskStore>):Void {
    sharedResolved = true;
    shared = store;
  }

  /** Whether every persistent hit must be recomputed and compared. */
  public static function verifying():Bool {
    var forced = verifyOverride;
    if (forced != null) return forced;
    var value = Sys.getEnv("MATERIA_MEMO_VERIFY");
    return value != null && value != "" && value != "0";
  }

  static function fromEnvironment():Null<DiskStore> {
    if (Sys.getEnv("MATERIA_MEMO") == "off") return null;
    var directory = Sys.getEnv("MATERIA_MEMO_DIR");
    if (directory == null || directory == "") {
      var cache = Sys.getEnv("XDG_CACHE_HOME");
      var home = Sys.getEnv("HOME");
      if (cache != null && cache != "") directory = cache + "/materia/memo";
      else if (home != null && home != "") directory = home + "/.cache/materia/memo";
      else return null;
    }
    var megabytes:Float = defaultCapMegabytes;
    var text = Sys.getEnv("MATERIA_MEMO_MAX_MB");
    if (text != null && text != "") {
      var parsed = Std.parseFloat(text);
      if (!Math.isNaN(parsed) && parsed >= 0) megabytes = parsed;
    }
    return new DiskStore(directory, megabytes * 1024 * 1024);
  }

  /** The file name for a key. */
  public function fileFor(key:String):String return directory + "/" + Sha256.encode(key) + ".memo";

  public function read(key:String):Null<Bytes> {
    var path = fileFor(key);
    try {
      var meta = FileSystem.metadata(path);
      if (meta == null) return null;
      var bytes = File.getBytes(path);
      if (Std.int(Sys.time()) - meta.modified > refreshSeconds) AtomicFile.writeBytes(path, bytes);
      return bytes;
    } catch (error:Dynamic) {
      return null;
    }
  }

  public function write(key:String, bytes:Bytes):Void {
    try {
      ensureDirectory();
      AtomicFile.writeBytes(fileFor(key), bytes);
      written += bytes.length;
      // Re-check the cap after about a tenth of it has been written since the last check, not on every write.
      if (!checked || written > capBytes / 10) {
        checked = true;
        written = 0;
        evict();
      }
    } catch (error:Dynamic) {}
  }

  /** Deletes every entry. */
  public function clear():Void {
    for (name in entries()) deleteQuietly(directory + "/" + name);
  }

  /** Total bytes of all entries. */
  public function usage():Float {
    var total = 0.0;
    for (name in entries()) {
      var meta = FileSystem.metadata(directory + "/" + name);
      if (meta != null) total += meta.size;
    }
    return total;
  }

  public function count():Int return entries().length;

  /** Drops the least recently used entries until the store is at 90 % of its cap (or under it already). */
  public function evict():Void {
    var items:Array<{path:String, size:Int, modified:Int}> = [];
    var total = 0.0;
    for (name in entries()) {
      var path = directory + "/" + name;
      var meta = FileSystem.metadata(path);
      if (meta == null) continue;
      items.push({path: path, size: meta.size, modified: meta.modified});
      total += meta.size;
    }
    if (total <= capBytes) return;
    items.sort((a, b) -> a.modified < b.modified ? -1 : a.modified > b.modified ? 1 : Reflect.compare(a.path, b.path));
    var target = capBytes * 0.9;
    for (item in items) {
      if (total <= target) break;
      deleteQuietly(item.path);
      total -= item.size;
    }
  }

  function entries():Array<String> {
    if (!FileSystem.exists(directory)) return [];
    var result = [];
    for (name in FileSystem.readDirectory(directory)) if (StringTools.endsWith(name, ".memo")) result.push(name);
    return result;
  }

  function ensureDirectory():Void {
    if (FileSystem.exists(directory)) return;
    var parts = directory.split("/");
    var path = "";
    for (part in parts) {
      path = path == "" ? (part == "" ? "/" : part) : (path == "/" ? "/" + part : path + "/" + part);
      if (part != "" && !FileSystem.exists(path)) FileSystem.createDirectory(path);
    }
  }

  static function deleteQuietly(path:String):Void {
    try FileSystem.deleteFile(path) catch (error:Dynamic) {}
  }
}

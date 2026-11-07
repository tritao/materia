package cadkit.cache;

import CadKit;
import cadkit.Shape;
import cadkit.ShapeKey;
import cadkit.Operation;
import haxe.io.Bytes;

/**
 * The persistent cache of boolean operations (`Shape.fuse`, `cut`, `common`) under construction keys (`ShapeKey`): the result of
 * a boolean is a function of the keys of its two inputs, the operation, the native build and this module's code, so it is kept
 * on disk and read back instead of computed. Nothing about the Haxe code that built the inputs matters: the same sequence of
 * operations gives the same keys, whatever source produced it.
 *
 * A keyed boolean always returns the CANONICAL form of its result (`Shape.canonical`, the form `toBytes` writes), on a miss as
 * on a hit and with the persistent tier off: a restored shape differs from a freshly built one in last-bit details, so a result
 * must never depend on whether the cache was consulted. An unkeyed boolean (an input with no key) is computed and returned as
 * built, as it always was.
 *
 * Environment (see `DiskStore`): `MATERIA_MEMO=off` (no persistent tier), `MATERIA_MEMO_DIR`, `MATERIA_MEMO_MAX_MB`,
 * `MATERIA_MEMO_VERIFY=1` (a hit is recomputed and compared; a difference throws).
 */
class OpCache {
  /** No initialisers: this class is reached from other classes' static initialisation. */
  static var counters:Null<Counters>;
  static var identityResolved = false;
  static var identity:Null<String>;

  static function stats():Counters {
    var known = counters;
    if (known == null) {
      known = new Counters();
      counters = known;
    }
    return known;
  }

  /** The text every stored operation is keyed under besides its construction key; null while no trustworthy identity exists. */
  static function tierIdentity():Null<String> {
    if (!identityResolved) {
      identityResolved = true;
      var native = NativeIdentity.get(), code = haxeon.build.CodeIdentity.ofModule("cadkit.Shape");
      identity = native == null || code == null ? null : "code:" + code + "|" + native;
    }
    return identity;
  }

  /** `compute()` for the boolean `op` of `a` and `b`; see the class comment. Takes no ownership of the inputs. */
  public static function boolean(op:String, a:Shape, b:Shape, compute:() -> Shape):Shape {
    var key = ShapeKey.op(op, "", [a, b]);
    var counts = stats();
    if (key == null) {
      var gap = ShapeKey.gapOf(op, [a, b]);
      var began = Sys.time();
      var result = compute();
      counts.unkeyed(gap, Sys.time() - began, faces(a) + faces(b));
      result.key = null;
      result.gap = gap;
      return result;
    }
    var size = faces(a) + faces(b);
    var store = DiskStore.process(), tier = tierIdentity(), diskKey:Null<String> = null, stored:Null<Bytes> = null;
    if (store != null && tier != null) {
      diskKey = "cadkit-op/1\n" + tier + "\n" + key;
      stored = store.read(diskKey);
      if (stored != null && !DiskStore.verifying()) {
        var restored:Null<Shape> = null;
        try restored = Shape.fromBytes(stored) catch (error:Dynamic) {}
        if (restored != null) {
          counts.hit(size);
          restored.key = key;
          restored.gap = null;
          return restored;
        }
        stored = null;
      }
    }
    var began = Sys.time();
    var built = compute();
    var bytes:Bytes = null;
    var canonical:Shape = null;
    try {
      bytes = built.toBytes();
      canonical = Shape.fromBytes(bytes);
    } catch (error:Dynamic) {
      built.close();
      throw error;
    }
    built.close();
    counts.miss(Sys.time() - began, size);
    if (diskKey != null && store != null) {
      if (stored != null) {
        if (!same(stored, bytes)) {
          canonical.close();
          throw 'persistent shape operation "$op" is stale: the stored result differs from a recomputation (a dependency is missing from its key)';
        }
        counts.verified();
      } else store.write(diskKey, bytes);
    }
    canonical.key = key;
    canonical.gap = null;
    return canonical;
  }

  /** Runs a history-keeping boolean (its result is built, uncached) and counts its time; `unkeyed` says whether an input had no key. */
  public static function historyOperation(op:String, gap:Null<String>, run:() -> Operation):Operation {
    var began = Sys.time();
    var result = run();
    stats().history(op, gap, Sys.time() - began);
    return result;
  }

  static function faces(shape:Shape):Int return shape.subshapeCount(CadKit.ShapeKind.Face);

  static function same(a:Bytes, b:Bytes):Bool {
    if (a.length != b.length) return false;
    for (index in 0...a.length) if (a.get(index) != b.get(index)) return false;
    return true;
  }

  /** For tests: zero the counters. */
  public static function resetStats():Void counters = null;

  /** A snapshot of the counters. */
  public static function snapshot():OpStats return stats().snapshot();

  /** One line per category: keyed hits and misses, unkeyed booleans by reason, history-keeping booleans. */
  public static function report():Array<String> {
    var s = stats().snapshot();
    var lines = ['keyed booleans: ${s.hits} hits, ${s.misses} misses (${Math.round(s.missSeconds * 1000)} ms computing and canonicalising), ${s.verified} verified',
      'unkeyed booleans: ${s.unkeyedCount} (${Math.round(s.unkeyedSeconds * 1000)} ms)',
      'history booleans: ${s.historyKeyed} keyed + ${s.historyUnkeyed} unkeyed (${Math.round(s.historySeconds * 1000)} ms, not cached: callers keep the operation history)'];
    for (gap => entry in s.gaps) lines.push('  unkeyed because "$gap": ${entry.count} (${Math.round(entry.seconds * 1000)} ms)');
    for (gap => entry in s.historyGaps) lines.push('  history unkeyed because "$gap": ${entry.count} (${Math.round(entry.seconds * 1000)} ms)');
    for (bucket => entry in s.sizes) lines.push('  size ${bucket}: ${entry.count} keyed, ${Math.round(entry.seconds * 1000)} ms computing');
    return lines;
  }
}

typedef OpGap = {count:Int, seconds:Float};

typedef OpStats = {
  var hits:Int;
  var misses:Int;
  var verified:Int;
  var missSeconds:Float;
  var unkeyedCount:Int;
  var unkeyedSeconds:Float;
  var historyKeyed:Int;
  var historyUnkeyed:Int;
  var historySeconds:Float;
  var gaps:Map<String, OpGap>;
  var sizes:Map<String, OpGap>;
  var historyGaps:Map<String, OpGap>;
}

private class Counters {
  var hitCount = 0;
  var missCount = 0;
  var verifiedCount = 0;
  var missTime = 0.0;
  var unkeyedCount = 0;
  var unkeyedTime = 0.0;
  var historyKeyed = 0;
  var historyUnkeyed = 0;
  var historyTime = 0.0;
  final gaps:Map<String, OpGap> = [];
  final sizes:Map<String, OpGap> = [];
  final historyGaps:Map<String, OpGap> = [];
  final lock = new sys.thread.Mutex();

  public function new() {}

  static function bucket(faces:Int):String return faces < 16 ? "<16 faces" : faces < 64 ? "16-63 faces" : faces < 256 ? "64-255 faces" : "256+ faces";

  public function hit(faces:Int):Void {
    lock.acquire();
    hitCount++;
    lock.release();
  }

  public function miss(seconds:Float, faces:Int):Void {
    lock.acquire();
    missCount++;
    missTime += seconds;
    var entry = sizes.get(bucket(faces));
    if (entry == null) {
      entry = {count: 0, seconds: 0.0};
      sizes.set(bucket(faces), entry);
    }
    entry.count++;
    entry.seconds += seconds;
    lock.release();
  }

  public function verified():Void {
    lock.acquire();
    verifiedCount++;
    lock.release();
  }

  public function unkeyed(gap:String, seconds:Float, faces:Int):Void {
    lock.acquire();
    unkeyedCount++;
    unkeyedTime += seconds;
    var entry = gaps.get(gap);
    if (entry == null) {
      entry = {count: 0, seconds: 0.0};
      gaps.set(gap, entry);
    }
    entry.count++;
    entry.seconds += seconds;
    lock.release();
  }

  public function history(op:String, gap:Null<String>, seconds:Float):Void {
    lock.acquire();
    if (gap != null) {
      historyUnkeyed++;
      var entry = historyGaps.get(gap);
      if (entry == null) {
        entry = {count: 0, seconds: 0.0};
        historyGaps.set(gap, entry);
      }
      entry.count++;
      entry.seconds += seconds;
    } else historyKeyed++;
    historyTime += seconds;
    lock.release();
  }

  public function snapshot():OpStats {
    lock.acquire();
    var result:OpStats = {hits: hitCount, misses: missCount, verified: verifiedCount, missSeconds: missTime, unkeyedCount: unkeyedCount,
      unkeyedSeconds: unkeyedTime, historyKeyed: historyKeyed, historyUnkeyed: historyUnkeyed, historySeconds: historyTime,
      gaps: [for (k => v in gaps) k => {count: v.count, seconds: v.seconds}],
      sizes: [for (k => v in sizes) k => {count: v.count, seconds: v.seconds}],
      historyGaps: [for (k => v in historyGaps) k => {count: v.count, seconds: v.seconds}]};
    lock.release();
    return result;
  }
}

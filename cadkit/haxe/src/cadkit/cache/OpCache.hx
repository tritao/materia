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
  public static function boolean(op:String, a:Shape, b:Shape, compute:() -> Shape):Shape return construct(
    op,
    "",
    [a, b],
    compute
  );

  /**
   * `compute()` for the construction `op` (with parameter text `params`) of `inputs`: a boolean, or another pure native
   * constructor whose result is a function of its inputs' keys (a loft). The result is the canonical form, kept on disk under
   * the construction key; see the class comment. Takes no ownership of the inputs. The op name must be one that names only this
   * result form (a history-keeping variant of the same operation uses another name).
   */
  /** The failure `Shape.toBytes` reports for a shape that does not settle, and the coverage gap named after it. */
  static final noCanonicalForm = "did not reach a canonical form";
  static final markerText = "cadkit-op: no canonical form";

  /** For tests: makes every canonicalization fail as a shape that never settles does. */
  public static var simulateNoCanonicalForm = false;

  static function isMarker(bytes:Bytes):Bool {
    if (bytes.length != markerText.length) return false;
    for (index in 0...bytes.length) if (bytes.get(index) != markerText.charCodeAt(index)) return false;
    return true;
  }

  public static function construct(op:String, params:String, inputs:Array<Shape>, compute:() -> Shape):Shape {
    var key = ShapeKey.op(op, params, inputs);
    var counts = stats();
    var size = 0;
    for (input in inputs) size += faces(input);
    if (key == null) {
      var gap = ShapeKey.gapOf(op, inputs);
      var began = Sys.time();
      var result = compute();
      counts.unkeyed(gap, Sys.time() - began, size);
      result.key = null;
      result.gap = gap;
      return result;
    }
    var store = DiskStore.process(), tier = tierIdentity(), diskKey:Null<String> = null, stored:Null<Bytes> = null;
    if (store != null && tier != null) {
      diskKey = "cadkit-op/1\n" + tier + "\n" + key;
      stored = store.read(diskKey);
      if (stored != null && isMarker(stored)) {
        // This result has no canonical form (see below): it is built every time, as the history-keeping operation would.
        var began = Sys.time();
        var result = compute();
        counts.unkeyed(noCanonicalForm, Sys.time() - began, size);
        result.key = null;
        result.gap = noCanonicalForm;
        return result;
      }
      if (stored != null && !DiskStore.verifying()) {
        var restored:Null<Shape> = null;
        try restored = Shape.fromBytes(stored) catch (error:Dynamic) {
        }
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
      if (simulateNoCanonicalForm) throw noCanonicalForm;
      bytes = built.toBytes();
      canonical = Shape.fromBytes(bytes);
    } catch (error:Dynamic) {
      if (Std.string(error).indexOf(noCanonicalForm) < 0) {
        built.close();
        throw error;
      }
      // Some results (an imported supplier solid cut by a slab, say) never settle into a byte-stable form. They cannot be
      // kept or restored exactly, so they are not cached: the built shape is returned unkeyed, which is what the
      // history-keeping boolean gave. A marker spares later runs the 32 write-and-read passes that found it out.
      counts.unkeyed(noCanonicalForm, Sys.time() - began, size);
      if (diskKey != null && store != null && stored == null) store.write(diskKey, Bytes.ofString(markerText));
      built.key = null;
      built.gap = noCanonicalForm;
      return built;
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

  static final queryIdentities:Map<String, Null<String>> = [];

  /** The text a stored query is keyed under besides its shape key: the code of the module that computes it and the native build. */
  static function queryIdentity(module:String):Null<String> {
    if (queryIdentities.exists(module)) return queryIdentities.get(module);
    var native = NativeIdentity.get(), code = haxeon.build.CodeIdentity.ofModule(module);
    var result = native == null || code == null ? null : "code:" + code + "|" + native;
    queryIdentities.set(module, result);
    return result;
  }

  /** The store key of query `name` (implemented by `module`) of `shape` with `params`; null for an unkeyed shape or no trustworthy identity. For tests. */
  public static function queryKey(name:String, module:String, shape:Shape, params:String):Null<String> {
    var key = shape.key, tier = queryIdentity(module);
    return key == null || tier == null ? null : "cadkit-query/1\n" + name + "\n" + tier + "\n" + params + "\n" + key;
  }

  /**
   * A pure query of a keyed shape (mass properties, face descriptors): its answer is a function of the shape's construction
   * key, `params`, the native build and the code of `module`, so it is kept on disk and read back instead of computed. Only
   * queries that do not depend on what was done to the shape's topology before (meshing it, for one) may use this: a query
   * whose answer changes with that history (a tessellation, a bounding box) is not a function of the key.
   *
   * An unkeyed shape, `MATERIA_MEMO=off` or an unknown identity compute as always. `encode` and `decode` must round-trip
   * exactly. With `MATERIA_MEMO_VERIFY=1` a stored answer is recomputed and compared; a difference throws.
   */
  public static function query<T>(
    name:String,
    module:String,
    shape:Shape,
    params:String,
    compute:() -> T,
    encode:T -> Bytes,
    decode:Bytes -> T
  ):T {
    var store = DiskStore.process();
    var diskKey = store == null ? null : queryKey(name, module, shape, params);
    if (store == null || diskKey == null) return compute();
    var counts = stats();
    var stored = store.read(diskKey);
    if (stored != null && !DiskStore.verifying()) {
      var decoded:Null<{value:T}> = null;
      try decoded = {value: decode(stored)} catch (error:Dynamic) {
      }
      if (decoded != null) {
        counts.queryHit(name);
        return decoded.value;
      }
      stored = null;
    }
    var began = Sys.time();
    var value = compute();
    var seconds = Sys.time() - began;
    var bytes = encode(value);
    if (stored != null) {
      if (!same(stored,
        bytes)) throw 'persistent shape query "$name" is stale: the stored answer differs from a recomputation (a dependency is missing from its key)';
      counts.queryVerified(name);
    } else store.write(diskKey, bytes);
    counts.queryMiss(name, seconds);
    return value;
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
    var lines = [
      'keyed booleans: ${s.hits} hits, ${s.misses} misses (${Math.round(s.missSeconds * 1000)} ms computing and canonicalising), ${s.verified} verified',
      'unkeyed booleans: ${s.unkeyedCount} (${Math.round(s.unkeyedSeconds * 1000)} ms)',
      'history booleans: ${s.historyKeyed} keyed + ${s.historyUnkeyed} unkeyed (${Math.round(s.historySeconds * 1000)} ms, not cached: callers keep the operation history)'
    ];
    for (name => q in s.queries) lines
      .push('query $name: ${q.hits} hits, ${q.misses} misses (${Math.round(q.seconds * 1000)} ms computing), ${q.verified} verified');
    for (gap => entry in s
      .gaps) lines.push('  unkeyed because "$gap": ${entry.count} (${Math.round(entry.seconds * 1000)} ms)');
    for (gap => entry in s.historyGaps) lines
      .push('  history unkeyed because "$gap": ${entry.count} (${Math.round(entry.seconds * 1000)} ms)');
    for (bucket => entry in s
      .sizes) lines.push('  size ${bucket}: ${entry.count} keyed, ${Math.round(entry.seconds * 1000)} ms computing');
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
  var gaps:Map<String,
  OpGap>;
  var sizes:Map<String,
  OpGap>;
  var historyGaps:Map<String,
  OpGap>;
  var queries:Map<String,
  OpQuery>;
}

typedef OpQuery = {hits:Int, misses:Int, verified:Int, seconds:Float};

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
  final queries:Map<String, OpQuery> = [];
  final lock = new sys.thread.Mutex();

  public function new() {
  }

  static function bucket(faces:Int):String return faces
    < 16 ? "<16 faces" : faces < 64 ? "16-63 faces" : faces < 256 ? "64-255 faces" : "256+ faces";

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

  function queryEntry(name:String):OpQuery {
    var entry = queries.get(name);
    if (entry == null) {
      entry = {hits: 0, misses: 0, verified: 0, seconds: 0.0};
      queries.set(name, entry);
    }
    return entry;
  }

  public function queryHit(name:String):Void {
    lock.acquire();
    queryEntry(name).hits++;
    lock.release();
  }

  public function queryMiss(name:String, seconds:Float):Void {
    lock.acquire();
    var entry = queryEntry(name);
    entry.misses++;
    entry.seconds += seconds;
    lock.release();
  }

  public function queryVerified(name:String):Void {
    lock.acquire();
    queryEntry(name).verified++;
    lock.release();
  }

  public function snapshot():OpStats {
    lock.acquire();
    var result:OpStats = {hits: hitCount, misses: missCount, verified: verifiedCount,
      missSeconds: missTime, unkeyedCount: unkeyedCount, unkeyedSeconds: unkeyedTime, historyKeyed: historyKeyed,
      historyUnkeyed: historyUnkeyed, historySeconds: historyTime, gaps: [for (k => v in gaps) k => {
      count:v.count, seconds:v.seconds
    }
    ], sizes:[for (k => v in sizes) k => {
      count:v.count, seconds:v.seconds
    }
    ], historyGaps:[for (k => v in historyGaps) k => {
      count:v.count, seconds:v.seconds
    }
    ], queries:[for (k => v in queries) k => {
      hits:v.hits, misses:v.misses, verified:v.verified, seconds:v.seconds
    }
    ]};
    lock.release();
    return result;
  }
}

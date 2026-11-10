import cadkit.Shape;
import cadkit.cache.DiskStore;
import cadkit.cache.OpCache;
import gearkit.*;
import haxe.io.Bytes;
import machinekit.hobbing.*;

/**
 * The standing gate for the kernel cache on a real machine component: a small hob (gashes cut and fused into a bored cylinder)
 * is built cold into a fresh store, then warm, then warm with verify on (every hit recomputed and compared).
 *
 * - Every boolean of the model is keyed: a result that loses its key (a new operation without one, a key that stops covering an
 *   input) shows as an unkeyed boolean and fails here.
 * - The warm run finds every operation in the store and does no work; the hit count may not fall below `BASELINE_HITS`, which
 *   catches a key that got worse without making anything impure.
 * - The verified run recomputes every hit and finds the same bytes: a result that depends on something its key does not cover
 *   throws there. Two controls prove the gate can fail: an operation made impure on purpose is caught by verify, and a boolean
 *   on an unkeyed shape is counted.
 *
 * When the model's operations change on purpose, run it and set `BASELINE_HITS` to the warm hit count it reports.
 */
class CacheGateTests {
  /** Keyed booleans a warm run of `model()` finds in the store. */
  static inline var BASELINE_HITS = 10;

  static var checks = 0;

  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }

  static function hex(bytes:Bytes):String {
    var digest = haxe.crypto.Sha256.make(bytes), out = new StringBuf();
    for (i in 0...digest.length) out.add(StringTools.hex(digest.get(i), 2));
    return out.toString();
  }

  static function model():Shape {
    var spec = new GearSpec(1, 20, 8, new Involute(20 * Math.PI / 180));
    var design = new HobDesign(ConjugateRack.of(spec.profile()), 1, 32, 4, 0.12);
    var tool = Hob.fromDesign(design, spec, 4, 16, false, true);
    var hob = new Hob(tool.values().copy().setNumber("keywayWidth", 5).setNumber("surfaceTolerance", 0.2).setNumber("edgeTolerance", 0.02));
    var part = HobSolid.build(hob);
    var shape = part.shape;
    check(part.solidCount() == 1, "the model is one solid");
    return shape;
  }

  static function freshStore():DiskStore {
    var root = Sys.getEnv("TMPDIR");
    if (root == null || root == "") root = "/tmp";
    var store = new DiskStore(root + "/materia-cache-gate-" + Sys.getPid() + "-" + Std.random(1000000000), 1e9);
    store.clear();
    return store;
  }

  static function impureOperationIsCaught():Void {
    var calls = 0;
    var input = Shape.box(2, 2, 2);
    function impure():Shape return OpCache.construct("gate-impure", "", [input], () -> {
      calls++;
      return Shape.box(1 + calls, 1, 1);
    });
    impure();
    var caught = false;
    DiskStore.forceVerify(true);
    try impure() catch (_:Dynamic) caught = true;
    DiskStore.forceVerify(null);
    check(caught, "verify throws for a keyed operation whose result depends on something its key does not cover");
  }

  static function unkeyedBooleanIsCounted():Void {
    OpCache.resetStats();
    var loose = Shape.box(2, 2, 2).unkeyed("gate-control");
    loose.fuse(Shape.box(1, 1, 5));
    check(OpCache.snapshot().unkeyedCount == 1, "a boolean on an unkeyed shape is counted as unkeyed");
  }

  public static function main():Void {
    var store = freshStore();
    DiskStore.useProcess(store);
    try {
      OpCache.resetStats();
      var began = Sys.time();
      var cold = model(), reference = hex(cold.toBytes());
      var c = OpCache.snapshot();
      Sys.println('cache gate cold: ${c.misses} misses, ${c.hits} hits, ${c.unkeyedCount} unkeyed, ${c.historyKeyed}+${c.historyUnkeyed} history (${Math.round((Sys.time() - began) * 1000)} ms)');
      check(c.misses > 0, "the cold run computes keyed booleans");
      check(c.historyUnkeyed == 0, 'every history-keeping boolean of the model is keyed (unkeyed: ${c.historyUnkeyed}; ' + OpCache.report().join("; ") + ")");
      check(c.unkeyedCount == 0, 'every boolean of the model is keyed (unkeyed: ${c.unkeyedCount}; ' + OpCache.report().join("; ") + ")");

      OpCache.resetStats();
      began = Sys.time();
      var warm = model();
      var w = OpCache.snapshot();
      Sys.println('cache gate warm: ${w.misses} misses, ${w.hits} hits (${Math.round((Sys.time() - began) * 1000)} ms)');
      check(w.misses == 0, 'a warm run finds every operation in the store (${w.misses} misses)');
      check(w.hits >= c.misses, 'the warm run hits what the cold run stored (${w.hits} vs ${c.misses})');
      check(w.hits >= BASELINE_HITS, 'warm hits ${w.hits} are not below the baseline $BASELINE_HITS');
      check(hex(warm.toBytes()) == reference, "the warm result is the cold result");

      DiskStore.forceVerify(true);
      OpCache.resetStats();
      began = Sys.time();
      var verified = model();
      var v = OpCache.snapshot();
      DiskStore.forceVerify(null);
      Sys.println('cache gate verify: ${v.verified} verified (${Math.round((Sys.time() - began) * 1000)} ms)');
      check(v.verified >= w.hits, 'verify recomputed every hit (${v.verified} of ${w.hits})');
      check(hex(verified.toBytes()) == reference, "the verified result is the cold result");

      impureOperationIsCaught();
      unkeyedBooleanIsCounted();
    } catch (error:Dynamic) {
      DiskStore.forceVerify(null);
      DiskStore.useProcess(null);
      store.clear();
      throw error;
    }
    DiskStore.useProcess(null);
    store.clear();
    Sys.println('Cache gate: $checks checks passed');
  }
}

package machinekit.cache;

import cadkit.Shape;
import cadkit.modeling.Part;
import machinekit.cache.Memo.MemoStats;
import machinekit.cache.Memo.MemoTable;

/** Every memo in the process, for profiling (`report`) and tests (`clearAll`). Memos join when they are built. */
class Memos {
  /** No initialiser: memos register from other classes' static initialisers, which may run before this class's. */
  static var tables:Null<Array<MemoTable>>;

  /** Native solids, each caller getting its own deep copy (see `Memo`). */
  public static function parts(name:String, capacity:Int):machinekit.cache.Memo<Part> return new machinekit.cache.Memo<Part>(name, capacity, part -> new Part(independent(part.shape)),
    part -> part.close());

  /**
   * Native solids kept on disk too (see `Memo.persist`): the key must name every input (a component's `geometryKey`), `module`
   * is the qualified name of the module whose code computes them. Values are the canonical form (`Shape.canonical`) on a miss,
   * with the tier on or off.
   */
  public static function persistentParts(name:String, capacity:Int, module:String):machinekit.cache.Memo<Part> return parts(name, capacity)
    .canonicalize(canonicalPart).persist(CodeIdentity.of(module), part -> part.shape.toBytes(), bytes -> new Part(Shape.fromBytes(bytes)));

  static function canonicalPart(part:Part):Part {
    try {
      var restored = part.shape.canonical();
      part.close();
      return new Part(restored);
    } catch (error:Dynamic) {
      part.close();
      throw error;
    }
  }

  /** Native shapes, each caller getting its own deep copy (see `Memo`). */
  public static function shapes(name:String, capacity:Int):machinekit.cache.Memo<Shape> return new machinekit.cache.Memo<Shape>(name, capacity, shape -> independent(shape),
    shape -> shape.close());

  /**
   * A copy of `shape` with topology of its own and the same geometry, names and surface attributes. The copy is the shape
   * moved by nothing with copying asked for (OCCT's transform with `copy`), which rebuilds every vertex, edge and face and drops
   * any triangulation; the coordinates are unchanged to the bit.
   */
  public static function independent(shape:Shape):Shape return shape.translate(new cadkit.modeling.Vector(0, 0, 0).native());

  /**
   * The native build every persistent value came from: the CadKit libraries this program loads (their content), the OCCT
   * prefix in use (`CADKIT_OCCT_DIR`, whose directory name carries OCCT's revision) and `MATERIA_MEMO_SALT` (set it after
   * changing something the other two cannot see). Null when the program has no CadKit build beside it, which turns the
   * persistent tier off.
   */
  public static function nativeIdentity():Null<String> {
    var cadkit = haxeon.build.CodeIdentity.nativeBuild("cadkit");
    if (cadkit == null) return null;
    var occt = Sys.getEnv("CADKIT_OCCT_DIR"), salt = Sys.getEnv("MATERIA_MEMO_SALT");
    return "cadkit:" + cadkit + (occt == null ? "" : "|occt:" + occt) + (salt == null ? "" : "|salt:" + salt) + "|naming:" + Shape.namingScheme();
  }

  public static function register(table:MemoTable):Void {
    var known = tables;
    if (known == null) {
      known = [];
      tables = known;
    }
    known.push(table);
  }

  public static function all():Array<MemoStats> {
    var known = tables;
    if (known == null) return [];
    var result = [for (table in known) table.stats()];
    result.sort((a, b) -> Reflect.compare(a.name, b.name));
    return result;
  }

  public static function clearAll():Void {
    var known = tables;
    if (known != null) for (table in known) table.clear();
  }

  /** One line per memo: hits, misses, evictions, entries and the time spent computing misses. */
  public static function report():Array<String> return [for (s in all()) s.name + ": " + s.hits + " hits, " + s.misses
    + " misses (" + s.diskHits + " from disk), " + s.evictions + " evicted, " + s.size + "/" + s.capacity + " kept, " + Math.round(s.computeSeconds * 1000)
    + " ms computing"];
}

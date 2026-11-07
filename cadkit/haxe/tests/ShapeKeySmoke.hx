import CadKit;
import cadkit.Geometry;
import cadkit.Shape;
import cadkit.cache.DiskStore;
import cadkit.cache.OpCache;
import cadkit.modeling.Curve;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import haxe.io.Bytes;
import sys.FileSystem;

/** Construction-history keys and the persistent boolean cache: keys name how a shape was made; hits equal recomputations. */
class ShapeKeySmoke {
  static var passed = 0;
  static var serial = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw "shape key: " + message;
    passed++;
  }

  static function hex(b:Bytes):String {
    var out = new StringBuf();
    for (i in 0...b.length) out.add(StringTools.hex(b.get(i), 2));
    return out.toString();
  }

  static function freshStore():DiskStore {
    serial++;
    var root = Sys.getEnv("TMPDIR");
    if (root == null || root == "") root = "/tmp";
    var store = new DiskStore(root + "/materia-shapekey-test-" + Sys.getPid() + "-" + serial, 1e9);
    store.clear();
    return store;
  }

  static function negativeZero():Float {
    return 1.0 / Math.NEGATIVE_INFINITY;
  }

  static function v(x:Float, y:Float, z:Float):CadKit.Vec3 return Geometry.vec3(x, y, z);

  /** A drilled block: a box less two cylinders, built only from keyed operations. */
  static function block(width:Float):Shape {
    var box = Shape.box(width, 20, 30);
    var moved = box.translate(v(0, 0, 0));
    box.close();
    var hole = Shape.cylinder(4, 40).translate(v(2, 3, -5));
    var other = Shape.cylinder(2, 40).translate(v(-3, -6, -5));
    var holes = hole.fuse(other);
    var result = moved.cut(holes);
    for (s in [moved, hole, other, holes]) s.close();
    return result;
  }

  static function keys():Void {
    var a = Shape.box(1, 2, 3), b = Shape.box(1, 2, 3), c = Shape.box(1, 2, 4);
    check(a.key != null && a.key == b.key, "equal construction, equal keys");
    check(a.key != c.key, "another parameter, another key");
    check(Shape.cylinder(1, 2).key != Shape.box(1, 1, 2).key, "another primitive, another key");
    check(a.translate(v(0.0, 0, 0)).key != a.translate(v(negativeZero(), 0, 0)).key, "signed zero is taken as bits (a miss, never a wrong hit)");
    var t1 = a.translate(v(1, 2, 3)), t2 = b.translate(v(1, 2, 3)), t3 = b.translate(v(1, 2, 4));
    check(t1.key == t2.key && t1.key != t3.key && t1.key != a.key, "translate keys follow input and parameters");
    var f1 = a.fuse(t1), f2 = b.fuse(t2);
    check(f1.key != null && f1.key == f2.key, "equal boolean, equal key");
    check(a.cut(t1).key != f1.key, "another operation, another key");
    check(t1.fuse(a).key != f1.key, "operand order matters");
    // Names are part of the history.
    var n1 = a.stamped("alpha", []), n2 = a.stamped("beta", []), n3 = b.stamped("alpha", []);
    check(n1.key != null && n1.key == n3.key && n1.key != n2.key, "stamp tags are part of the key");
    check(a.withElementNames(CadKit.ShapeKind.Face, ["x", "", "", "", "", ""]).key != a.withElementNames(CadKit.ShapeKind.Face, ["y", "", "", "", "", ""]).key,
      "seeded names are part of the key");
    check(a.cloneShape().key == a.key, "a clone is the same shape");
    // Placement and parts.
    var p1 = Part.box(1, 2, 3), p2 = Part.box(1, 2, 3), p3 = Part.box(1, 2, 3, Min, Center, Max);
    check(p1.shape.key != null && p1.shape.key == p2.shape.key && p1.shape.key != p3.shape.key, "Part.box keys follow alignment");
    var l1 = p1.placed(Location.translation(new Vector(1, 2, 3))), l2 = p2.placed(Location.translation(new Vector(1, 2, 3)));
    check(l1.shape.key != null && l1.shape.key == l2.shape.key && l1.shape.key != p1.shape.key, "placement keys");
    var pr1 = Part.prism([new Vector(0, 0), new Vector(5, 0), new Vector(5, 3)], 0, 2),
      pr2 = Part.prism([new Vector(0, 0), new Vector(5, 0), new Vector(5, 3)], 0, 2),
      pr3 = Part.prism([new Vector(0, 0), new Vector(5, 0), new Vector(5, 4)], 0, 2);
    check(pr1.shape.key != null && pr1.shape.key == pr2.shape.key && pr1.shape.key != pr3.shape.key, "prism keys follow the polygon");
    check(Part.cylinderAlongY(2, 0, 5, 1, 1).shape.key != null, "cylinderAlongY is keyed");
    check(p1.named("one").shape.key != p1.named("two").shape.key, "Part.named tags differ");
    // Unkeyed sources and propagation.
    var spline = Curve.spline([new Vector(0, 0, 0), new Vector(1, 1, 0), new Vector(2, 0, 0)]);
    check(spline.shape.key == null && spline.shape.gap == "spline", "a spline has no key and says why");
    var raw = Shape.fromOwnedHandle(CadKit.boxChecked(1, 1, 1));
    check(raw.key == null, "a raw native shape has no key");
    check(raw.translate(v(1, 0, 0)).key == null && raw.fuse(a).key == null && a.cut(raw).key == null, "null propagates through operations");
    check(Shape.compound([a, b]).key == null, "a compound has no key");
    check(a.fillet(0.1).key != null && a.fillet(0.1).key == b.fillet(0.1).key, "whole-shape fillet is keyed");
    var cs = Curve.polyline([new Vector(0, 0, 0), new Vector(1, 0, 0)]), ds = Curve.polyline([new Vector(0, 0, 0), new Vector(1, 0, 0)]);
    check(cs.shape.key == ds.shape.key, "equal polylines, equal keys");
  }

  static function cache():Void {
    var store = freshStore();
    DiskStore.useProcess(store);
    OpCache.resetStats();
    var first = block(10);
    var bytes = first.toBytes();
    var s1 = OpCache.snapshot();
    check(first.key != null && s1.misses == 2 && s1.hits == 0, "two keyed booleans, both misses: " + s1.misses + "/" + s1.hits);
    check(store.count() == 2, "each stored once: " + store.count());
    check(hex(first.canonical().toBytes()) == hex(bytes), "a keyed result is already the canonical form");
    var second = block(10);
    var s2 = OpCache.snapshot();
    check(s2.misses == 2 && s2.hits == 2, "the second build runs no boolean: " + s2.misses + "/" + s2.hits);
    check(hex(second.toBytes()) == hex(bytes) && second.key == first.key, "and returns the same bytes and key");
    check(hex(block(11).toBytes()) != hex(bytes), "another parameter is another result");
    // The persistent tier off gives the same bytes.
    DiskStore.useProcess(null);
    var off = block(10);
    check(hex(off.toBytes()) == hex(bytes), "with the tier off the result is the same canonical bytes");
    // A hit returns what the store holds; verify mode refuses a planted wrong entry.
    DiskStore.useProcess(store);
    var files = [for (name in FileSystem.readDirectory(store.directory)) if (StringTools.endsWith(name, ".memo")) name];
    check(files.length == 3, "three entries now (block 11 added one cut; its fuse of the holes hit): " + files.length);
    var planted = Shape.box(7, 7, 7).toBytes();
    var victim = block(10);
    OpCache.resetStats();
    // Overwrite every entry with a wrong shape: the next build reads it back.
    for (name in files) sys.io.File.saveBytes(store.directory + "/" + name, planted);
    var fooled = block(10);
    check(OpCache.snapshot().hits >= 1 && hex(fooled.toBytes()) != hex(bytes), "a hit returns what is stored (so the key must be right)");
    DiskStore.forceVerify(true);
    var message = "";
    try block(10) catch (error:Dynamic) message = Std.string(error);
    DiskStore.forceVerify(null);
    check(message.indexOf("stale") >= 0, "verify mode refuses a planted wrong entry: " + message);
    store.clear();
    DiskStore.forceVerify(true);
    var honest = block(10);
    var verified = OpCache.snapshot().verified;
    var again = block(10);
    DiskStore.forceVerify(null);
    check(hex(honest.toBytes()) == hex(bytes) && hex(again.toBytes()) == hex(bytes), "verify mode accepts honest entries");
    check(OpCache.snapshot().verified > verified, "and counts the verified hits");
    store.clear();
    DiskStore.useProcess(null);
  }

  static function parts():Void {
    var store = freshStore();
    DiskStore.useProcess(store);
    OpCache.resetStats();
    function build():Part {
      var base = Part.box(20, 20, 10);
      var tools = [Part.cylinderSpan(3, -1, 12, 4, 4), Part.cylinderSpan(2, -1, 12, -4, -4), Part.cylinderAlongY(1.5, -15, 15, 0, 5)];
      var result = base.subtractAll(tools);
      base.close();
      for (t in tools) t.close();
      return result;
    }
    var a = build();
    check(a.shape.key != null && a.solidCount() == 1, "subtractAll of keyed parts is keyed");
    var s1 = OpCache.snapshot();
    check(s1.misses > 0 && s1.unkeyedCount == 0, "its booleans were cached ones: " + s1.misses + "/" + s1.unkeyedCount);
    var b = build();
    var s2 = OpCache.snapshot();
    check(s2.misses == s1.misses && s2.hits == s1.misses && hex(a.shape.toBytes()) == hex(b.shape.toBytes()), "a rebuild is all hits with the same bytes");
    var fused = Part.fuseAll([Part.box(1, 1, 1), Part.box(1, 1, 1).translated(new Vector(5, 0, 0)), Part.box(2, 2, 2).translated(new Vector(0, 5, 0))]);
    check(fused.shape.key != null && fused.solidCount() == 3, "fuseAll of keyed parts is keyed");
    // combine keeps its operation history (the parametric layer relies on it).
    var combined = Part.box(2, 2, 2).combine(Part.box(1, 1, 1), Subtract);
    check(combined.operation != null && combined.shape.key != null, "combine keeps its history and is keyed");
    store.clear();
    DiskStore.useProcess(null);
  }

  public static function main():Void run();

  public static function run():Void {
    keys();
    cache();
    parts();
    Sys.println("shape key smoke: " + passed + " checks passed");
  }
}

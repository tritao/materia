import CadKit;
import cadkit.Geometry;
import cadkit.Shape;
import cadkit.cache.DiskStore;
import cadkit.Mesh;
import cadkit.ShapeKey;
import cadkit.cache.OpCache;
import cadkit.modeling.Curve;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.parametric.GeometricConnectors;
import cadkit.PhysicalProperties;
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
    check(spline.shape.key != null && spline.shape.key == Curve.spline([new Vector(0, 0, 0), new Vector(1, 1, 0), new Vector(2, 0, 0)]).shape.key,
      "equal splines, equal keys");
    check(spline.shape.key != Curve.spline([new Vector(0, 0, 0), new Vector(1, 1, 0), new Vector(2, 0, 1)]).shape.key, "another point, another spline key");
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


  /** Sections like the hob's: closed polylines at stepped heights and twists. */
  static function sections(twist:Float):Array<Curve> {
    var out:Array<Curve> = [];
    for (i in 0...4) {
      var z = i * 3.0, r = 4 + 0.2 * i, a = twist * i;
      var points = [for (k in 0...6) new Vector(r * Math.cos(a + k * Math.PI / 3), r * Math.sin(a + k * Math.PI / 3), z)];
      out.push(Curve.polyline(points, true));
    }
    return out;
  }

  static function closeAll(curves:Array<Curve>):Void for (c in curves) c.close();

  static function loftClip(twist:Float):Part {
    var s = sections(twist);
    var base = Part.loft(s, false);
    closeAll(s);
    var blank = Part.cylinderSpan(6, -1, 11);
    var clipped = base.combineKept(blank, cadkit.modeling.Mode.Intersect);
    var tool = Part.cylinderSpan(1, -2, 12, 0.5, 0);
    var result = clipped.subtractAll([tool]);
    base.close();
    blank.close();
    clipped.close();
    tool.close();
    return result;
  }

  /** Lofts and splines are keyed; a clipped loft is cached like any other keyed boolean. */
  static function lofts():Void {
    var a = sections(0.1), b = sections(0.1), c = sections(0.2);
    var la = Part.loft(a, false), lb = Part.loft(b, false), lc = Part.loft(c, false), lr = Part.loft(sections(0.1), true);
    check(la.shape.key != null && la.shape.key == lb.shape.key, "equal sections, equal loft keys");
    check(la.shape.key != lc.shape.key, "other sections, another loft key");
    check(la.shape.key != lr.shape.key, "the ruled flag is part of the loft key");
    var reordered = sections(0.1);
    reordered.reverse();
    check(Part.loft(reordered, false).shape.key != la.shape.key, "section order is part of the loft key");
    // A section with no key makes an unkeyed loft, with the reason carried along.
    var rawSection = new Curve(Shape.fromOwnedHandle(CadKit.polylineChecked([v(0, 0, 0), v(2, 0, 0), v(2, 2, 0), v(0, 2, 0)], 1)));
    var other = Curve.polyline([new Vector(0, 0, 3), new Vector(2, 0, 3), new Vector(2, 2, 3), new Vector(0, 2, 3)], true);
    var unkeyed = Part.loft([rawSection, other], false);
    check(unkeyed.shape.key == null, "a loft with an unkeyed section is unkeyed");
    // combineKept: cached when keyed, exactly `combine` (with history) when not.
    var kept = la.combineKept(Part.cylinderSpan(6, -1, 11), cadkit.modeling.Mode.Intersect);
    check(kept.shape.key != null && kept.operation == null, "combineKept of keyed parts is a keyed result without history");
    var plain = unkeyed.combineKept(Part.box(1, 1, 1), cadkit.modeling.Mode.Add);
    check(plain.operation != null && plain.shape.key == null, "combineKept of an unkeyed part is combine, with its history");
    closeAll(a);
    closeAll(b);
    closeAll(c);
    // Cached: the second build runs no boolean and gives the same bytes; with the tier off the bytes are the same again.
    var store = freshStore();
    DiskStore.useProcess(store);
    OpCache.resetStats();
    var first = loftClip(0.1);
    var s1 = OpCache.snapshot();
    check(first.shape.key != null && s1.misses >= 2 && s1.unkeyedCount == 0, "a clipped loft was all cached booleans: " + s1.misses + "/" + s1.unkeyedCount);
    var bytes = first.shape.toBytes();
    var second = loftClip(0.1);
    var s2 = OpCache.snapshot();
    check(s2.misses == s1.misses && s2.hits == s1.misses, "the rebuild ran no boolean: " + s2.misses + "/" + s2.hits);
    check(hex(second.shape.toBytes()) == hex(bytes) && second.shape.key == first.shape.key, "and gave the same bytes and key");
    check(hex(loftClip(0.2).shape.toBytes()) != hex(bytes), "another twist is another result");
    DiskStore.useProcess(null);
    var off = loftClip(0.1);
    check(hex(off.shape.toBytes()) == hex(bytes), "with the tier off the clipped loft has the same canonical bytes");
    DiskStore.forceVerify(true);
    DiskStore.useProcess(store);
    try {
      var verified = loftClip(0.1);
      check(hex(verified.shape.toBytes()) == hex(bytes), "verify mode accepts a clipped loft");
    } catch (error:Dynamic) {
      DiskStore.forceVerify(null);
      DiskStore.useProcess(null);
      throw error;
    }
    DiskStore.forceVerify(null);
    DiskStore.useProcess(null);
    store.clear();
  }

  static function sameProps(a:PhysicalProperties, b:PhysicalProperties):Bool return hex(a.toBytes()) == hex(b.toBytes());

  static function queryStats(name:String):{hits:Int, misses:Int, verified:Int, seconds:Float} {
    var q = OpCache.snapshot().queries.get(name);
    return q == null ? {hits: 0, misses: 0, verified: 0, seconds: 0.0} : q;
  }

  /** Mass properties and face descriptors of keyed shapes: kept on disk, equal to a recomputation, independent of meshing. */
  static function queries():Void {
    var store = freshStore();
    DiskStore.useProcess(store);
    OpCache.resetStats();
    // The answers do not depend on whether the shape was meshed: that is what lets them be a function of the key.
    var probe = block(10), meshed = block(10);
    meshed.tessellate(0.1, 0.5);
    DiskStore.useProcess(null);
    var plainProps = probe.massProperties(), meshedProps = meshed.massProperties();
    var plainFaces = GeometricConnectors.describeFaces(probe), meshedFaces = GeometricConnectors.describeFaces(meshed);
    check(sameProps(plainProps, meshedProps), "mass properties are the same before and after meshing");
    check(plainFaces == meshedFaces, "face descriptors are the same before and after meshing");
    DiskStore.useProcess(store);
    OpCache.resetStats();
    // Mass properties.
    var a = block(10), b = block(10), c = block(11);
    var base = store.count();
    var p1 = a.massProperties();
    check(queryStats("mass").misses == 1 && queryStats("mass").hits == 0 && store.count() == base + 1, "a first query is a miss and is stored");
    var p2 = b.massProperties();
    check(queryStats("mass").misses == 1 && queryStats("mass").hits == 1, "the same construction, another shape object: a hit, no recomputation");
    check(sameProps(p1, p2) && sameProps(p1, plainProps), "a hit equals the computed answer to the bit");
    c.massProperties();
    check(queryStats("mass").misses == 2 && store.count() == base + 2, "another shape is another entry");
    // The tier off, an unkeyed shape: computed as always, nothing stored or counted.
    DiskStore.useProcess(null);
    check(sameProps(block(10).massProperties(), p1), "with the tier off the answer is the same");
    DiskStore.useProcess(store);
    var unkeyed = block(10).unkeyed("test");
    var before = queryStats("mass");
    check(sameProps(unkeyed.massProperties(), p1) && queryStats("mass").hits == before.hits && queryStats("mass").misses == before.misses && store.count() == base + 2,
      "an unkeyed shape bypasses the cache");
    // Face descriptors.
    var f1 = GeometricConnectors.describeFaces(a);
    check(queryStats("describe-faces").misses == 1 && f1 == plainFaces, "descriptors: first query is a miss");
    var f2 = GeometricConnectors.describeFaces(b);
    check(queryStats("describe-faces").hits == 1 && f2 == f1, "descriptors: a hit equals the computed text");
    check(GeometricConnectors.describeFaces(c) != f1 && queryStats("describe-faces").misses == 2, "descriptors: another shape, another entry");
    DiskStore.useProcess(null);
    check(GeometricConnectors.describeFaces(block(10)) == f1, "descriptors: tier off, same text");
    DiskStore.useProcess(store);
    // A hit returns what is stored; verify mode accepts an honest entry and refuses a planted one.
    DiskStore.forceVerify(true);
    var honest = block(10).massProperties();
    DiskStore.forceVerify(null);
    check(sameProps(honest, p1) && queryStats("mass").verified == 1, "verify mode accepts an honest entry");
    var kmass = OpCache.queryKey("mass", "cadkit.Shape", a, ""), kfaces = OpCache.queryKey("describe-faces", "cadkit.parametric.GeometricConnectors", a, "");
    check(kmass != null && kfaces != null && kmass != kfaces, "each query has its own store key");
    check(OpCache.queryKey("mass", "cadkit.Shape", unkeyed, "") == null, "an unkeyed shape has no store key");
    store.write(kmass, c.massProperties().toBytes());
    DiskStore.forceVerify(true);
    var message = "";
    try block(10).massProperties() catch (error:Dynamic) message = Std.string(error);
    DiskStore.forceVerify(null);
    check(message.indexOf("stale") >= 0, "verify mode refuses a planted wrong mass entry: " + message);
    store.write(kfaces, haxe.io.Bytes.ofString("[]"));
    DiskStore.forceVerify(true);
    message = "";
    try GeometricConnectors.describeFaces(block(10)) catch (error:Dynamic) message = Std.string(error);
    DiskStore.forceVerify(null);
    check(message.indexOf("stale") >= 0, "verify mode refuses planted wrong descriptors: " + message);
    // Damaged bytes are a miss, recomputed and rewritten.
    store.write(kmass, haxe.io.Bytes.ofString("junk"));
    check(sameProps(block(10).massProperties(), p1), "a damaged entry is a miss");
    check(sameProps(block(10).massProperties(), p1), "and was rewritten whole");
    store.clear();
    DiskStore.useProcess(null);
  }

  static function meshHex(m:Mesh):String return hex(m.toBytes());

  static function boundsText(s:Shape):String {
    var b = s.bounds(), lo = b.get_min(), hi = b.get_max();
    return [lo.get_x(), lo.get_y(), lo.get_z(), hi.get_x(), hi.get_y(), hi.get_z()].join(",");
  }

  /** Area of the triangles a mesh puts in the range of face `index`. */
  static function rangeArea(m:Mesh, first:Int, count:Int):Float {
    var total = 0.0;
    var t = 0;
    while (t < count) {
      var pts = [for (k in 0...3) {
        var vi = m.indices.getInt32((first + t + k) * 4);
        [for (c in 0...3) m.vertices.getDouble((vi * 3 + c) * 8)];
      }];
      var ux = pts[1][0] - pts[0][0], uy = pts[1][1] - pts[0][1], uz = pts[1][2] - pts[0][2];
      var wx = pts[2][0] - pts[0][0], wy = pts[2][1] - pts[0][1], wz = pts[2][2] - pts[0][2];
      var cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
      total += 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
      t += 3;
    }
    return total;
  }

  /** Tessellation is pure (meshes a copy) and a keyed shape's mesh is kept on disk. */
  static function tessellation():Void {
    DiskStore.useProcess(null);
    // Purity: the shape is not changed by meshing, and a mesh does not depend on what was meshed before.
    var s = block(10);
    var before = boundsText(s);
    var fine = s.tessellate(0.01, 0.5);
    check(boundsText(s) == before, "meshing leaves bounds() as they were");
    var coarse = s.tessellate(1.0, 0.5);
    check(meshHex(coarse) == meshHex(block(10).tessellate(1.0, 0.5)), "a coarse mesh after a fine one equals a fresh coarse mesh");
    check(meshHex(fine) == meshHex(block(10).tessellate(0.01, 0.5)), "and the fine one equals a fresh fine mesh");
    check(s.tessellate(1.0, 0.5) == coarse, "the same request returns the same mesh");
    var rel = s.tessellateRelative(0.005, 0.5);
    check(meshHex(rel) == meshHex(block(10).tessellateRelative(0.005, 0.5)), "a relative mesh does not depend on earlier meshes");
    var ball = Shape.sphere(7);
    var tight = boundsText(ball);
    ball.tessellate(0.001, 0.2);
    check(boundsText(ball) == tight, "a sphere's bounds stay tight after meshing");
    // The mesh's face indices are the shape's own: each range's triangles have the area of that face.
    var worst = 0.0;
    for (r in fine.faceRanges) {
      var face = s.subshape(CadKit.ShapeKind.Face, r.faceIndex);
      var area = face.area();
      worst = Math.max(worst, Math.abs(rangeArea(fine, r.firstIndex, r.indexCount) - area) / Math.max(area, 1e-9));
    }
    check(fine.faceRanges.length == s.subshapeCount(CadKit.ShapeKind.Face) && worst < 0.01, "mesh face ranges map to the shape's faces by area: " + worst);
    var edgeCount = s.subshapeCount(CadKit.ShapeKind.Edge), badEdge = false;
    for (i in 0...Std.int(fine.edgeIds.length / 4)) {
      var id = fine.edgeIds.getInt32(i * 4);
      if (id < 0 || id >= edgeCount) badEdge = true;
    }
    check(!badEdge, "mesh edge ids are the shape's edge indices");
    // The mesh round-trips as bytes, exactly.
    var back = Mesh.fromBytes(fine.toBytes());
    check(meshHex(back) == meshHex(fine) && back.vertexCount == fine.vertexCount && back.indexCount == fine.indexCount
      && back.linearDeflection == fine.linearDeflection && back.faceRanges.length == fine.faceRanges.length, "a mesh round-trips through bytes in every field");
    var failed = false;
    try Mesh.fromBytes(Bytes.ofString("xx")) catch (error:Dynamic) failed = true;
    check(failed, "a short mesh buffer is refused");
    var extra = fine.toBytes(), padded = Bytes.alloc(extra.length + 1);
    padded.blit(0, extra, 0, extra.length);
    failed = false;
    try Mesh.fromBytes(padded) catch (error:Dynamic) failed = true;
    check(failed, "a mesh buffer with trailing bytes is refused");
    // The persistent tier.
    var store = freshStore();
    DiskStore.useProcess(store);
    OpCache.resetStats();
    var a = block(10), b = block(10), c = block(11);
    var m1 = a.tessellate(0.05, 0.5);
    check(queryStats("tessellate").misses == 1 && queryStats("tessellate").hits == 0, "a first mesh is a miss");
    var m2 = b.tessellate(0.05, 0.5);
    check(queryStats("tessellate").hits == 1 && queryStats("tessellate").misses == 1, "the same construction, another shape object: a hit, no tessellation");
    check(meshHex(m1) == meshHex(m2), "a hit equals the computed mesh to the bit");
    a.tessellate(0.05, 0.5);
    check(queryStats("tessellate").hits == 1, "the shape's own memory answers a repeat without touching the store");
    a.tessellate(0.5, 0.5);
    check(queryStats("tessellate").misses == 2, "other deflections are other entries");
    a.tessellate(0.05, 0.25);
    check(queryStats("tessellate").misses == 3, "another angular deflection is another entry");
    c.tessellate(0.05, 0.5);
    check(queryStats("tessellate").misses == 4, "another shape is another entry");
    var relA = a.tessellateRelative(0.01, 0.5), relB = block(10).tessellateRelative(0.01, 0.5);
    check(meshHex(relA) == meshHex(relB) && queryStats("tessellate").hits >= 2, "relative meshes are cached by their resolved deflection");
    DiskStore.useProcess(null);
    check(meshHex(block(10).tessellate(0.05, 0.5)) == meshHex(m1), "with the tier off the mesh is the same");
    DiskStore.useProcess(store);
    var unkeyed = block(10).unkeyed("test");
    var seen = queryStats("tessellate");
    check(meshHex(unkeyed.tessellate(0.05, 0.5)) == meshHex(m1) && queryStats("tessellate").hits == seen.hits && queryStats("tessellate").misses == seen.misses,
      "an unkeyed shape bypasses the cache and gives the same mesh");
    // Verify mode and damage.
    DiskStore.forceVerify(true);
    var honest = block(10).tessellate(0.05, 0.5);
    DiskStore.forceVerify(null);
    check(meshHex(honest) == meshHex(m1) && queryStats("tessellate").verified == 1, "verify mode accepts an honest entry");
    var params = ShapeKey.number(0.05) + "," + ShapeKey.number(0.5);
    var key = OpCache.queryKey("tessellate", "cadkit.Shape", a, params);
    check(key != null, "a keyed shape has a mesh store key");
    store.write(key, c.tessellate(0.05, 0.5).toBytes());
    DiskStore.forceVerify(true);
    var message = "";
    try block(10).tessellate(0.05, 0.5) catch (error:Dynamic) message = Std.string(error);
    DiskStore.forceVerify(null);
    check(message.indexOf("stale") >= 0, "verify mode refuses a planted wrong mesh: " + message);
    store.write(key, Bytes.ofString("junk"));
    check(meshHex(block(10).tessellate(0.05, 0.5)) == meshHex(m1), "a damaged entry is a miss");
    check(meshHex(block(10).tessellate(0.05, 0.5)) == meshHex(m1), "and was rewritten whole");
    store.clear();
    DiskStore.useProcess(null);
  }

  /** `canonical` is an operation on the key: another shape to anything keyed on how it was made. */
  static function canonicalKeys():Void {
    var a = block(10), b = block(10);
    var ca = a.canonical(), cb = b.canonical();
    check(ca.key != null && ca.key != a.key, "the canonical form has its own key");
    check(ca.key == cb.key, "equal construction, equal canonical keys");
    check(block(11).canonical().key != ca.key, "another shape, another canonical key");
    check(ca.canonical().key != ca.key && ca.canonical().key == ca.canonical().key, "canonical of canonical is a further, deterministic key");
    var un = Shape.box(1, 2, 3).unkeyed("test").canonical();
    check(un.key == null, "an unkeyed shape stays unkeyed");
    // Queries on a canonical keyed shape are cached under its own key.
    var store = freshStore();
    DiskStore.useProcess(store);
    OpCache.resetStats();
    var m1 = ca.massProperties(), m2 = cb.massProperties();
    check(queryStats("mass").misses == 1 && queryStats("mass").hits == 1 && sameProps(m1, m2), "queries on canonical keyed shapes are cached");
    store.clear();
    DiskStore.useProcess(null);
  }

  public static function main():Void run();

  public static function run():Void {
    keys();
    cache();
    parts();
    lofts();
    queries();
    canonicalKeys();
    tessellation();
    Sys.println("shape key smoke: " + passed + " checks passed");
  }
}

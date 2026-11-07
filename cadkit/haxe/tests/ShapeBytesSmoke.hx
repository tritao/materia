import CadKit;
import cadkit.Geometry;
import cadkit.Shape;
import cadkit.threads.SurfaceDetail;
import cadkit.threads.SurfaceDetail.DisplayBinding;
import cadkit.threads.SurfaceDetail.DisplaySpec;
import cadkit.threads.ThreadSpec;
import cadkit.threads.ThreadedFace;
import haxe.io.Bytes;

/** Shapes as bytes (OC5): a restored shape equals the original in everything a cache consumer reads. */
class ShapeBytesSmoke {
  static var passed = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw "shape bytes: " + message;
    passed++;
  }

  static function kinds():Array<CadKit.ShapeKind> return [
    CadKit.ShapeKind.Solid,
    CadKit.ShapeKind.Face,
    CadKit.ShapeKind.Edge,
    CadKit.ShapeKind.Vertex
  ];

  static function sameBytes(a:Bytes, b:Bytes):Bool return a.length == b.length && a.compare(b) == 0;

  static function near(a:Float, b:Float, tolerance:Float):Bool return a == b || Math.abs(a - b) <= tolerance * Math.max(
    1,
    Math.max(Math.abs(a), Math.abs(b))
  );

  static function attributeText(shape:Shape):String {
    var parts:Array<String> = [];
    for (attribute in shape.faceAttributes()) {
      var spec = attribute.spec;
      parts.push([
        attribute.faceName,
        spec.designation,
        Std.string(spec.majorDiameter),
        Std.string(spec.pitch),
        Std.string(spec.starts),
        Std.string(spec.hand),
        Std.string(spec.internal),
        spec.toleranceClass,
        Std.string(spec.profile),
        Std.string(attribute.start),
        Std.string(attribute.direction),
        Std.string(attribute.length),
        describe(attribute.detail)
      ].join("|"));
    }
    return parts.join("\n");
  }

  static function describe(detail:Null<SurfaceDetail>):String return detail == null ? "-" : switch detail {
    case Knurl(teeth, depth):
      'knurl $teeth $depth';
    case Graduation(divisions, numberedEvery, slant, endRadius):
      'grad $divisions $numberedEvery $slant $endRadius';
    case Display(spec):
      'display ${spec.width} ${spec.height} ${spec.offsetX} ${spec.offsetY} ${spec.source} ${spec.profile} ${[for (b in spec.bindings) b.signal + "=" + b.joint].join(",")} ${spec.channel} ${spec.round}';
  };

  /** Doubles equal as numbers: a signed zero is the same value as a plain one. */
  static function sameDoubles(a:Bytes, b:Bytes, tolerance:Float = 0):Bool {
    if (a.length != b.length) return false;
    for (index in 0...Std.int(a.length / 8)) {
      var x = a.getDouble(index * 8), y = b.getDouble(index * 8);
      if (x != y && Math.abs(x - y) > tolerance) return false;
    }
    return true;
  }

  /** The distinct vertices of a mesh to a micron, sorted, as one text. */
  static function vertexSet(vertices:Bytes):String {
    var keys:Map<String, Bool> = [];
    for (i in 0...Std.int(vertices.length / 24)) keys.set(
      [for (k in 0...3) Std.string(Math.round(vertices.getDouble(i * 24 + k * 8) * 1e6) + 0.0)].join(","),
      true
    );
    var list = [for (key in keys.keys()) key];
    list.sort(Reflect.compare);
    return list.join(";");
  }

  /** Two meshes of one canonical form: every byte equal. */
  static function sameMeshBytes(a:Shape, b:Shape, what:String):Void {
    var ma = a.tessellate(0.05, 0.3), mb = b.tessellate(0.05, 0.3);
    check(
      sameBytes(ma.vertices, mb.vertices) && sameBytes(ma.normals, mb.normals) && sameBytes(ma.indices, mb.indices),
      what + ": canonical meshes are byte-identical"
    );
    check(
      sameBytes(ma.edgeSegments, mb.edgeSegments) && sameBytes(ma.edgeIds, mb.edgeIds),
      what + ": canonical edge segments are byte-identical"
    );
  }

  /**
   * Everything observable, before either shape has been meshed. `a` is the original and `b` its canonical form: they agree in
   * every number but possibly the sign of a zero (and, where a rotation is stored in a location, the last bit); `tolerance`
   * bounds that for such shapes, zero for the rest.
   */
  static function sameShape(a:Shape, b:Shape, what:String, withMesh:Bool = true, tolerance:Float = 0):Void {
    for (kind in kinds()) {
      check(a.subshapeCount(kind) == b.subshapeCount(kind), what + ": " + kind + " count");
      check(a.elementNames(kind).join("\n") == b.elementNames(kind).join("\n"), what + ": " + kind + " names");
    }
    for (kind in [CadKit.ShapeKind.Face, CadKit.ShapeKind.Solid]) check(
      [for (list in a.elementAliases(kind)) list.join(",")].join(";")
      == [for (list in b.elementAliases(kind)) list.join(",")].join(";"),
      what + ": " + kind + " aliases"
    );
    check(a.kind() == b.kind(), what + ": kind");
    if (a.subshapeCount(CadKit.ShapeKind.Vertex) > 0) {
      var boundsA = a.bounds(), boundsB = b.bounds();
      check(
        near(boundsA.get_min().get_x(), boundsB.get_min().get_x(), tolerance) && near(
          boundsA.get_min().get_y(),
          boundsB.get_min().get_y(),
          tolerance
        ) && near(
          boundsA.get_min().get_z(),
          boundsB.get_min().get_z(),
          tolerance
        ) && near(
          boundsA.get_max().get_x(),
          boundsB.get_max().get_x(),
          tolerance
        ) && near(
          boundsA.get_max().get_y(),
          boundsB.get_max().get_y(),
          tolerance
        ) && near(
          boundsA.get_max().get_z(),
          boundsB.get_max().get_z(),
          tolerance
        ),
        what + ": bounds"
      );
    }
    if (a.hasSolidBoundary()) {
      check(near(a.volume(), b.volume(), tolerance), what + ": volume " + a.volume() + " vs " + b.volume());
      check(near(a.area(), b.area(), tolerance), what + ": area");
      var pa = a.massProperties(), pb = b.massProperties();
      check(
        near(pa.centerOfMass.x,
        pb.centerOfMass.x, tolerance) && near(pa.centerOfMass.y, pb.centerOfMass.y, tolerance) && near(
          pa.centerOfMass.z,
          pb.centerOfMass.z,
          tolerance
        ),
        what + ": centre of mass"
      );
    }
    check(attributeText(a) == attributeText(b), what + ": face attributes");
    if (withMesh && a.hasSolidBoundary()) {
      var ma = a.tessellate(0.05, 0.3), mb = b.tessellate(0.05, 0.3);
      check(ma.vertexCount == mb.vertexCount && ma.indexCount == mb.indexCount, what + ": mesh size");
      if (tolerance == 0) {
        check(
          sameDoubles(ma.vertices, mb.vertices) && sameDoubles(ma.normals, mb.normals) && sameBytes(
            ma.indices,
            mb.indices
          ),
          what + ": mesh values"
        );
        check(
          sameDoubles(ma.edgeSegments, mb.edgeSegments) && sameBytes(ma.edgeIds, mb.edgeIds),
          what + ": edge segments"
        );
      } else {
        // A rotation stored in a location is rebuilt by the reader, and a cylinder's triangulation can then start elsewhere:
        // the same vertices in another order. The canonical form is what is deterministic (see sameMeshBytes).
        check(
          ma.indexCount == mb.indexCount && vertexSet(ma.vertices) == vertexSet(mb.vertices),
          what + ": the same mesh vertices"
        );
      }
    }
  }

  static function roundTrip(shape:Shape, what:String, withMesh:Bool = true, tolerance:Float = 0):Shape {
    var bytes = shape.toBytes();
    var restored = Shape.fromBytes(bytes);
    check(sameBytes(bytes, shape.toBytes()), what + ": writing twice gives the same bytes");
    // OCCT's reader rebuilds locations and surface placements from stored values (signed zeros, last bits), over several passes
    // for some shapes. toBytes writes the fixed point, so what it hands out reads back to a shape that writes exactly it again.
    var again = Shape.fromBytes(bytes);
    var canonicalBytes = restored.toBytes();
    check(sameBytes(canonicalBytes, bytes), what + ": the restored shape writes exactly the bytes it was read from");
    check(sameBytes(again.toBytes(), bytes), what + ": restoring the same bytes twice gives the same shape");
    var second = Shape.fromBytes(canonicalBytes);
    var secondBytes = second.toBytes();
    check(sameBytes(secondBytes, canonicalBytes), what + ": write-then-read is a fixed point");
    var third = Shape.fromBytes(secondBytes);
    check(sameBytes(third.toBytes(), secondBytes), what + ": and stays one");
    var canonical = shape.canonical();
    check(sameBytes(canonical.toBytes(), canonicalBytes), what + ": canonical() is the restored shape");
    var twice = canonical.canonical();
    check(sameBytes(twice.toBytes(), canonicalBytes), what + ": canonical() of a canonical shape changes nothing");
    // Bounds depend on whether OCCT has meshed a shape's topology, so everything unmeshed is compared first.
    sameShape(shape, restored, what, withMesh, tolerance);
    if (withMesh && shape.hasSolidBoundary()) {
      sameMeshBytes(restored, again, what);
      sameMeshBytes(restored, second, what + " (second pass)");
      sameMeshBytes(restored, twice, what + " (canonical twice)");
    }
    for (extra in [again, second, third, canonical, twice]) extra.close();
    // Meshing may enlarge a few edge/vertex tolerances (OCCT writes them into the B-rep), so bytes should be taken before
    // meshing; triangulations themselves are never written, and original and restored follow each other through meshing.
    var meshed = shape.toBytes();
    check(meshed.length == bytes.length, what + ": meshing leaves the size alone");
    check(
      meshed.length == restored.toBytes().length,
      what + ": original and restored write alike in size after both were meshed"
    );
    return restored;
  }

  static function fails(action:() -> Void, what:String):Void {
    var failed = false;
    try action() catch (error:Dynamic) failed = true;
    check(failed, what + " is refused");
  }

  static function primitives():Void {
    for (entry in [
      {name: "box", shape: Shape.box(10, 20, 30)},
      {name: "cylinder", shape: Shape.cylinder(7, 25)},
      {name: "sphere", shape: Shape.sphere(9)}
    ]) {
      var restored = roundTrip(entry.shape, entry.name);
      restored.close();
      entry.shape.close();
    }
  }

  static function features():Void {
    var box = Shape.box(40, 30, 20);
    var hole = Shape.cylinder(4, 40).translate(Geometry.vec3(10, 10, -5));
    var drilled = box.cut(hole);
    var filleted = drilled.fillet(1.5);
    var chamfered = Shape.box(20, 20, 20).chamfer(2);
    var stamped = Shape.box(10, 10, 10).stamped("f3", []);
    var fused = Shape.box(10, 20, 30).stamped("a", []).fuse(Shape.box(10, 20, 30).stamped("b", []));
    var moved = fused.translate(Geometry.vec3(5, -3, 2));
    var rotated = Shape.box(20, 10, 5).stamped("r", []).rotateOperation(Geometry.vec3(1, 1, 0), 0.7).resultShape();
    var tilted = Shape.cylinder(4, 15).stamped("t", []).rotateOperation(Geometry.vec3(0, 1, 1), 1.1).resultShape();
    for (entry in [
      {name: "drilled", shape: drilled},
      {name: "filleted", shape: filleted},
      {name: "chamfered", shape: chamfered},
      {name: "stamped", shape: stamped},
      {name: "fused with aliases", shape: fused},
      {name: "moved", shape: moved}
    ]) {
      var restored = roundTrip(entry.shape, entry.name);
      restored.close();
    }
    // Rotations live in locations, which OCCT's reader rebuilds from the stored matrix: values move by about 1e-16.
    for (entry in [{name: "rotated box", shape: rotated}, {name: "tilted cylinder", shape: tilted}]) {
      var restored = roundTrip(entry.shape, entry.name, true, 1e-12);
      restored.close();
    }
    check(
      fused.elementAliases(CadKit.ShapeKind.Face).join("").length > 0,
      "the fused shape really has aliases to preserve"
    );
    for (shape in [box, hole, drilled, filleted, chamfered, stamped, fused, moved, rotated, tilted]) shape.close();
  }

  static function compounds():Void {
    var a = Shape.box(10, 10, 10).stamped("p", []);
    var b = Shape.cylinder(3, 12).translate(Geometry.vec3(30, 0, 0)).stamped("q", []);
    var c = Shape.sphere(4).translate(Geometry.vec3(0, 30, 0));
    var compound = Shape.compound([a, b, c]);
    check(compound.subshapeCount(CadKit.ShapeKind.Solid) == 3, "a compound of three solids");
    var restored = roundTrip(compound, "compound");
    check(restored.subshapeCount(CadKit.ShapeKind.Solid) == 3, "the restored compound has three solids");
    var solid = restored.subshape(CadKit.ShapeKind.Solid, 1);
    var original = compound.subshape(CadKit.ShapeKind.Solid, 1);
    check(solid.volume() == original.volume(), "a solid taken from the restored compound matches");
    var empty = Shape.compound([]);
    var back = roundTrip(empty, "empty compound");
    check(
      back.subshapeCount(CadKit.ShapeKind.Face) == 0 && back.subshapeCount(CadKit.ShapeKind.Solid) == 0,
      "empty stays empty"
    );
    for (shape in [a, b, c, compound, restored, solid, original, empty, back]) shape.close();
  }

  static function attributes():Void {
    var cylinder = Shape.cylinder(5, 30);
    var spec = new ThreadSpec("M10", 10, 1.5, 2, LeftHand, true, "6H", Metric);
    var display = new DisplaySpec(
      20,
      10,
      1.5,
      -2.5,
      "lcd",
      "operator",
      [new DisplayBinding("x", "joint_x"), new DisplayBinding("y", "joint_y")],
      "main",
      true
    );
    var attributed = cylinder.cloneShape();
    attributed.setThreadedFaces([
      new ThreadedFace("cyl.side", spec, 2.5, -1, 12.25),
      new ThreadedFace("cyl.side", new ThreadSpec("M3", 3, 0.5), 0, 1, null),
      ThreadedFace.decoration("cyl.side", Knurl(40, 0.2), 1, 1, 8),
      ThreadedFace.decoration("cyl.side", Graduation(100, 10, 31.5, 5.5), 0, 1, null),
      ThreadedFace.decoration("face#0", Display(display), 0, 1, null)
    ]);
    var restored = roundTrip(attributed, "attributed");
    check(restored.faceAttributes().length == 5, "all five attributes survive");
    check(
      restored.threadedFaces().length == 2 && restored.surfaceDetails().length == 3,
      "threads and details stay separate"
    );
    // A fresh shape: cloneShape shares topology, and bounds depend on whether OCCT has meshed that topology.
    var fresh = Shape.cylinder(5, 30);
    var plain = roundTrip(fresh, "plain cylinder");
    check(plain.faceAttributes().length == 0, "no attributes stay none");
    for (shape in [cylinder, attributed, restored, fresh, plain]) shape.close();
  }

  /**
   * A boolean of keyed shapes returns the canonical form (`OpCache`): a fresh shape with topology of its own. One of restored
   * shapes (no key) is the built form, which shares topology with its inputs, so meshing an earlier result puts triangulations on
   * the inputs and moves the bounds of the next result by the mesh deflection. They agree to that, not bit for bit, so these
   * comparisons carry a tolerance.
   */
  static function booleans():Void {
    var base = Shape.box(30, 30, 30).stamped("base", []);
    var tool = Shape.cylinder(5, 50).translate(Geometry.vec3(5, 5, -10)).stamped("tool", []);
    var restoredBase = Shape.fromBytes(base.toBytes());
    var restoredTool = Shape.fromBytes(tool.toBytes());
    sameShape(base.cut(tool), restoredBase.cut(restoredTool), "cut", true, 1e-6);
    sameShape(base.fuse(tool), restoredBase.fuse(restoredTool), "fuse", true, 1e-6);
    sameShape(base.common(tool), restoredBase.common(restoredTool), "common", true, 1e-6);
    sameShape(base.cut(tool), restoredBase.cut(tool), "cut with one side restored", true, 1e-6);
    var slab = Shape.box(60, 8, 60).translate(Geometry.vec3(-10, 2, -10)).stamped("slab", []);
    var restoredSlab = Shape.fromBytes(slab.toBytes());
    sameShape(
      base.cut(tool).common(slab),
      restoredBase.cut(restoredTool).common(restoredSlab),
      "cut then common",
      true,
      1e-6
    );
    sameShape(base.fuse(tool).cut(slab),
      restoredBase.fuse(restoredTool).cut(restoredSlab), "fuse then cut", true, 1e-6);
    // Stamping and moving a restored shape names things as the original does.
    sameShape(
      base.translate(Geometry.vec3(1, 2, 3)).stamped("m", [base]),
      restoredBase.translate(Geometry.vec3(1, 2, 3)).stamped("m", [restoredBase]),
      "move and stamp"
    );
  }

  static function imported():Void {
    // A STEP round trip of a built part (no names, as an imported part has none), and a real supplier part if one is given.
    var root = Sys.getEnv("TMPDIR");
    if (root == null || root == "") root = "/tmp";
    var path = root + "/cadkit-shape-bytes-" + Sys.getPid() + ".step";
    var built = Shape.box(30, 20, 10).cut(Shape.cylinder(3, 30).translate(Geometry.vec3(8, 6, -5)));
    built.exportStep(path);
    var paths = [path];
    var supplied = Sys.getEnv("CADKIT_SHAPE_BYTES_STEP");
    if (supplied != null && supplied != "" && sys.FileSystem.exists(supplied)) paths.push(supplied);
    for (file in paths) {
      var part = Shape.importStep(file);
      var restored = roundTrip(part, "imported STEP part " + file.substr(file.lastIndexOf("/") + 1));
      check(restored.subshapeCount(CadKit.ShapeKind.Face) == part.subshapeCount(CadKit.ShapeKind.Face), "STEP faces");
      check(
        restored.elementNames(CadKit.ShapeKind.Face).join(",").indexOf("face#") >= 0,
        "an imported part keeps its weak index names"
      );
      part.close();
      restored.close();
    }
    sys.FileSystem.deleteFile(path);
    built.close();
  }

  static function hostile():Void {
    var shape = Shape.box(10, 20, 30).stamped("h", []);
    var bytes = shape.toBytes();
    fails(() -> Shape.fromBytes(Bytes.alloc(0)), "empty bytes");
    fails(() -> Shape.fromBytes(Bytes.ofString("not a shape at all, just text")), "text");
    fails(() -> Shape.fromBytes(Bytes.alloc(4096)), "zeros");
    for (cut in [1,
      3, 4, 8, 11, 12, 16, 40, bytes.length >> 1, bytes.length - 12, bytes.length - 5, bytes.length - 1]) fails(
        () -> Shape.fromBytes(Bytes.view(bytes, 0, cut)),
        'truncation to $cut of ${bytes.length} bytes'
      );
    // A flipped bit anywhere is caught: by the container, the native checksum, or the attribute checksum.
    var step = Std.int(Math.max(1, bytes.length / 60));
    var position = 0;
    var flips = 0;
    while (position < bytes.length) {
      var damaged = Bytes.alloc(bytes.length);
      damaged.blit(0, bytes, 0, bytes.length);
      damaged.set(position, damaged.get(position) ^ 0x10);
      var failed = false;
      try Shape.fromBytes(damaged) catch (error:Dynamic) failed = true;
      check(failed, 'a flipped bit at $position of ${bytes.length} is refused');
      position += step;
      flips++;
    }
    check(flips >= 60, "many positions were flipped");
    // Trailing bytes, and a restored shape after all those failures still works.
    var padded = Bytes.alloc(bytes.length + 1);
    padded.blit(0, bytes, 0, bytes.length);
    fails(() -> Shape.fromBytes(padded), "trailing byte");
    var fine = Shape.fromBytes(bytes);
    sameShape(shape, fine, "after refusals");
    shape.close();
    fine.close();
  }

  static function negativeZeros(bytes:Bytes):Int {
    var count = 0;
    for (index in 0...Std.int(bytes.length / 8)) {
      var value = bytes.getDouble(index * 8);
      if (value == 0 && 1 / value < 0) count++;
    }
    return count;
  }

  /**
   * Tessellation reports the geometry as it is, signed zeros included: a mesh is never normalised, so meshes of existing
   * shapes keep their bytes. (OC5 once rewrote -0.0 as +0.0 in every mesh to hide the reader's signed zeros; the cure is the
   * canonical form instead, and this keeps the tessellator from growing that rewrite again.)
   */
  static function tessellationKeepsSignedZeros():Void {
    var total = 0;
    for (shape in [Shape.cylinder(5, 30), Shape.sphere(9), Shape.box(10, 20, 30).fillet(1.5)]) {
      var mesh = shape.tessellate(0.05, 0.3);
      total += negativeZeros(mesh.vertices) + negativeZeros(mesh.normals) + negativeZeros(mesh.edgeSegments);
      shape.close();
    }
    Sys.println("negative zeros in built primitive meshes: " + total);
    check(total > 0, "built shapes mesh with signed zeros intact (" + total + ")");
  }

  public static function main():Void run();

  public static function run():Void {
    primitives();
    features();
    compounds();
    attributes();
    booleans();
    imported();
    hostile();
    tessellationKeepsSignedZeros();
    Sys.println("shape bytes smoke: " + passed + " checks passed");
  }
}

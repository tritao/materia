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

  static function kinds():Array<CadKit.ShapeKind> return [CadKit.ShapeKind.Solid, CadKit.ShapeKind.Face, CadKit.ShapeKind.Edge, CadKit.ShapeKind.Vertex];

  static function sameBytes(a:Bytes, b:Bytes):Bool return a.length == b.length && a.compare(b) == 0;

  static function attributeText(shape:Shape):String {
    var parts:Array<String> = [];
    for (attribute in shape.faceAttributes()) {
      var spec = attribute.spec;
      parts.push([attribute.faceName, spec.designation, Std.string(spec.majorDiameter), Std.string(spec.pitch), Std.string(spec.starts),
        Std.string(spec.hand), Std.string(spec.internal), spec.toleranceClass, Std.string(spec.profile), Std.string(attribute.start),
        Std.string(attribute.direction), Std.string(attribute.length), describe(attribute.detail)].join("|"));
    }
    return parts.join("\n");
  }

  static function describe(detail:Null<SurfaceDetail>):String return detail == null ? "-" : switch detail {
    case Knurl(teeth, depth): 'knurl $teeth $depth';
    case Graduation(divisions, numberedEvery, slant, endRadius): 'grad $divisions $numberedEvery $slant $endRadius';
    case Display(spec): 'display ${spec.width} ${spec.height} ${spec.offsetX} ${spec.offsetY} ${spec.source} ${spec.profile} ${[for (b in spec.bindings) b.signal + "=" + b.joint].join(",")} ${spec.channel} ${spec.round}';
  };

  /** Everything observable, before either shape has been meshed. */
  static function sameShape(a:Shape, b:Shape, what:String, withMesh:Bool = true):Void {
    for (kind in kinds()) {
      check(a.subshapeCount(kind) == b.subshapeCount(kind), what + ": " + kind + " count");
      check(a.elementNames(kind).join("\n") == b.elementNames(kind).join("\n"), what + ": " + kind + " names");
    }
    for (kind in [CadKit.ShapeKind.Face, CadKit.ShapeKind.Solid])
      check([for (list in a.elementAliases(kind)) list.join(",")].join(";") == [for (list in b.elementAliases(kind)) list.join(",")].join(";"),
        what + ": " + kind + " aliases");
    check(a.kind() == b.kind(), what + ": kind");
    if (a.subshapeCount(CadKit.ShapeKind.Vertex) > 0) {
      var boundsA = a.bounds(), boundsB = b.bounds();
      check(boundsA.get_min().get_x() == boundsB.get_min().get_x() && boundsA.get_min().get_y() == boundsB.get_min().get_y()
        && boundsA.get_min().get_z() == boundsB.get_min().get_z() && boundsA.get_max().get_x() == boundsB.get_max().get_x()
        && boundsA.get_max().get_y() == boundsB.get_max().get_y() && boundsA.get_max().get_z() == boundsB.get_max().get_z(), what + ": bounds");
    }
    if (a.hasSolidBoundary()) {
      check(a.volume() == b.volume(), what + ": volume " + a.volume() + " vs " + b.volume());
      check(a.area() == b.area(), what + ": area");
      var pa = a.massProperties(), pb = b.massProperties();
      check(pa.centerOfMass.x == pb.centerOfMass.x && pa.centerOfMass.y == pb.centerOfMass.y && pa.centerOfMass.z == pb.centerOfMass.z,
        what + ": centre of mass");
    }
    check(attributeText(a) == attributeText(b), what + ": face attributes");
    if (withMesh && a.hasSolidBoundary()) {
      var ma = a.tessellate(0.05, 0.3), mb = b.tessellate(0.05, 0.3);
      check(ma.vertexCount == mb.vertexCount && ma.indexCount == mb.indexCount, what + ": mesh size");
      check(sameBytes(ma.vertices, mb.vertices) && sameBytes(ma.normals, mb.normals) && sameBytes(ma.indices, mb.indices),
        what + ": mesh bytes");
      check(sameBytes(ma.edgeSegments, mb.edgeSegments) && sameBytes(ma.edgeIds, mb.edgeIds), what + ": edge segments");
    }
  }

  static function roundTrip(shape:Shape, what:String, withMesh:Bool = true):Shape {
    var bytes = shape.toBytes();
    var restored = Shape.fromBytes(bytes);
    check(sameBytes(bytes, shape.toBytes()), what + ": writing twice gives the same bytes");
    // Reading re-derives some internal curve data (signed zeros, last-bit values), so a restored shape may write different
    // bytes than the original; restoring the same bytes always gives the same shape, though, and everything observable matches.
    var again = Shape.fromBytes(bytes);
    check(sameBytes(restored.toBytes(), again.toBytes()), what + ": restoring the same bytes twice gives the same shape");
    check(restored.toBytes().length == bytes.length, what + ": the restored shape writes as many bytes");
    again.close();
    sameShape(shape, restored, what, withMesh);
    // Meshing may enlarge a few edge/vertex tolerances (OCCT writes them into the B-rep), so bytes should be taken before
    // meshing; triangulations themselves are never written, and original and restored follow each other through meshing.
    var meshed = shape.toBytes();
    check(meshed.length == bytes.length, what + ": meshing leaves the size alone");
    check(meshed.length == restored.toBytes().length, what + ": original and restored write alike in size after both were meshed");
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
    check(fused.elementAliases(CadKit.ShapeKind.Face).join("").length > 0, "the fused shape really has aliases to preserve");
    for (shape in [box, hole, drilled, filleted, chamfered, stamped, fused, moved]) shape.close();
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
    check(back.subshapeCount(CadKit.ShapeKind.Face) == 0 && back.subshapeCount(CadKit.ShapeKind.Solid) == 0, "empty stays empty");
    for (shape in [a, b, c, compound, restored, solid, original, empty, back]) shape.close();
  }

  static function attributes():Void {
    var cylinder = Shape.cylinder(5, 30);
    var spec = new ThreadSpec("M10", 10, 1.5, 2, LeftHand, true, "6H", Metric);
    var display = new DisplaySpec(20, 10, 1.5, -2.5, "lcd", "operator", [new DisplayBinding("x", "joint_x"), new DisplayBinding("y", "joint_y")], "main", true);
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
    check(restored.threadedFaces().length == 2 && restored.surfaceDetails().length == 3, "threads and details stay separate");
    // A fresh shape: cloneShape shares topology, and bounds depend on whether OCCT has meshed that topology.
    var fresh = Shape.cylinder(5, 30);
    var plain = roundTrip(fresh, "plain cylinder");
    check(plain.faceAttributes().length == 0, "no attributes stay none");
    for (shape in [cylinder, attributed, restored, fresh, plain]) shape.close();
  }

  static function booleans():Void {
    var base = Shape.box(30, 30, 30).stamped("base", []);
    var tool = Shape.cylinder(5, 50).translate(Geometry.vec3(5, 5, -10)).stamped("tool", []);
    var restoredBase = Shape.fromBytes(base.toBytes());
    var restoredTool = Shape.fromBytes(tool.toBytes());
    sameShape(base.cut(tool), restoredBase.cut(restoredTool), "cut");
    sameShape(base.fuse(tool), restoredBase.fuse(restoredTool), "fuse");
    sameShape(base.common(tool), restoredBase.common(restoredTool), "common");
    sameShape(base.cut(tool), restoredBase.cut(tool), "cut with one side restored");
    var slab = Shape.box(60, 8, 60).translate(Geometry.vec3(-10, 2, -10)).stamped("slab", []);
    var restoredSlab = Shape.fromBytes(slab.toBytes());
    sameShape(base.cut(tool).common(slab), restoredBase.cut(restoredTool).common(restoredSlab), "cut then common");
    sameShape(base.fuse(tool).cut(slab), restoredBase.fuse(restoredTool).cut(restoredSlab), "fuse then cut");
    // Stamping and moving a restored shape names things as the original does.
    sameShape(base.translate(Geometry.vec3(1, 2, 3)).stamped("m", [base]), restoredBase.translate(Geometry.vec3(1, 2, 3)).stamped("m", [restoredBase]),
      "move and stamp");
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
      check(restored.elementNames(CadKit.ShapeKind.Face).join(",").indexOf("face#") >= 0, "an imported part keeps its weak index names");
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
    for (cut in [1, 3, 4, 8, 11, 12, 16, 40, bytes.length >> 1, bytes.length - 12, bytes.length - 5, bytes.length - 1])
      fails(() -> Shape.fromBytes(Bytes.view(bytes, 0, cut)), 'truncation to $cut of ${bytes.length} bytes');
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

  public static function main():Void run();

  public static function run():Void {
    primitives();
    features();
    compounds();
    attributes();
    booleans();
    imported();
    hostile();
    Sys.println("shape bytes smoke: " + passed + " checks passed");
  }
}

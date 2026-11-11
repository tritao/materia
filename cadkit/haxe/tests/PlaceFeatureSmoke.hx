import cadkit.Shape;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ReferencePlaneElement;
import cadkit.parametric.features.BlockFeature;
import cadkit.parametric.features.PlaceFeature;
import cadkit.parametric.recording.DocumentBuilder;

/** A solid placed in a frame, driven by named dimensions and by a datum frame. */
class PlaceFeatureSmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "PlaceFeatureSmoke: " + message;
  }

  /** The box's eight corners taken through `frames` (innermost first), as min/max bounds. */
  static function expectedBounds(w:Float, d:Float, h:Float, frames:Array<Plane>):{min:Vector, max:Vector} {
    var lo = new Vector(1e300, 1e300, 1e300), hi = new Vector(-1e300, -1e300, -1e300);
    for (cx in [0.0, w]) for (cy in [0.0, d]) for (cz in [0.0, h]) {
      var p = new Vector(cx, cy, cz);
      for (frame in frames) p = frame.toWorld(p);
      lo = new Vector(Math.min(lo.x, p.x), Math.min(lo.y, p.y), Math.min(lo.z, p.z));
      hi = new Vector(Math.max(hi.x, p.x), Math.max(hi.y, p.y), Math.max(hi.z, p.z));
    }
    return {min: lo, max: hi};
  }

  static function checkBounds(shape:Shape, expected:{min:Vector, max:Vector}, what:String):Void {
    var b = shape.bounds(), lo = b.get_min(), hi = b.get_max();
    var got = [lo.get_x(), lo.get_y(), lo.get_z(), hi.get_x(), hi.get_y(), hi.get_z()];
    var want = [expected.min.x, expected.min.y, expected.min.z, expected.max.x, expected.max.y, expected.max.z];
    for (i in 0...6) check(Math.abs(got[i] - want[i]) < 1e-6, '$what bound $i: ${got[i]} vs ${want[i]}');
  }

  static function output(document:Document):Shape return document.outputFeature().currentShape();

  public static function run():Void {
    // An axis permutation, as machine placements usually are: part x along world y, part z along world x.
    var turn = new Plane(new Vector(100, -20, 50), new Vector(0, 1, 0), new Vector(1, 0, 0));
    var box:Null<BlockFeature> = null;
    var placed:Null<PlaceFeature> = null;
    var document = DocumentBuilder.build(function(builder) {
      var mountX = builder.length("mount.x", 100);
      box = builder.box(10, 20, 30);
      placed = builder.place(box, mountX, -20.0, 50.0, turn.xDirection, turn.normal);
      builder.output(placed);
    });
    if (box == null || placed == null) throw "PlaceFeatureSmoke: builder did not record";
    // BlockFeature's box spans [0, w] x [0, d] x [0, h] in its own frame.
    checkBounds(output(document), expectedBounds(10, 20, 30, [turn]), "placed box");
    check(Math.abs(output(document).volume() - 6000) < 1e-6, "volume kept");

    // Same result as placing the shape directly with Location, which PosedParts.moved uses today.
    var direct = new Location(turn).applyOperation(box.currentShape());
    var directShape = direct.resultShape();
    checkBounds(directShape, expectedBounds(10, 20, 30, [turn]), "direct placement");
    directShape.close();
    direct.close();

    // A named dimension drives the origin.
    document.parameter("mount.x").set(140);
    document.recompute();
    var moved = new Plane(new Vector(140, -20, 50), turn.xDirection, turn.normal);
    checkBounds(output(document), expectedBounds(10, 20, 30, [moved]), "after mount.x = 140");

    // Round trip: same file again, same geometry, binding intact.
    var text = DocumentCodec.encode(document);
    document.close();
    var restored = DocumentCodec.decode(text);
    check(DocumentCodec.encode(restored) == text, "save after restore is identical");
    checkBounds(output(restored), expectedBounds(10, 20, 30, [moved]), "restored");
    restored.parameter("mount.x").set(0);
    restored.recompute();
    checkBounds(output(restored), expectedBounds(10, 20, 30, [new Plane(new Vector(0, -20, 50), turn.xDirection, turn.normal)]),
      "restored binding drives the origin");
    restored.close();

    // Relative to a datum frame: the solid follows when the datum moves, and the datum survives the round trip.
    var seat = new Plane(new Vector(0, 0, 637), new Vector(0, -1, 0), new Vector(0, 0, 1));
    var local = new Plane(new Vector(5, 0, 0), new Vector(1, 0, 0), new Vector(0, 0, 1));
    var datum:Null<ReferencePlaneElement> = null;
    var relative = DocumentBuilder.build(function(builder) {
      datum = builder.datum("x-carriage seat", seat);
      var body = builder.box(10.0, 20.0, 30.0);
      builder.output(builder.placeIn(body, local, datum));
    });
    if (datum == null) throw "PlaceFeatureSmoke: no datum";
    checkBounds(output(relative), expectedBounds(10, 20, 30, [local, seat]), "relative to the datum");
    var raised = new Plane(new Vector(0, 0, 700), seat.xDirection, seat.normal);
    relative.setReferencePlane(datum, raised);
    relative.recompute();
    checkBounds(output(relative), expectedBounds(10, 20, 30, [local, raised]), "follows the moved datum");
    var relativeText = DocumentCodec.encode(relative);
    relative.close();
    var relativeRestored = DocumentCodec.decode(relativeText);
    check(DocumentCodec.encode(relativeRestored) == relativeText, "datum document saves identically after restore");
    checkBounds(output(relativeRestored), expectedBounds(10, 20, 30, [local, raised]), "restored relative placement");
    relativeRestored.close();

    Sys.println("PlaceFeatureSmoke: placement, named origin and datum frames pass");
  }

  public static function main():Void run();
}

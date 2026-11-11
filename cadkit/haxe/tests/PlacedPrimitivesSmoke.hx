import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.Feature;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.recording.DocumentBuilder;

/** Boxes from bounds, cylinders on an axis, cut-many and union-all as single recorded steps. */
class PlacedPrimitivesSmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "PlacedPrimitivesSmoke: " + message;
  }

  static function near(a:Float, b:Float, what:String):Void check(Math.abs(a - b) < 1e-6 * Math.max(1, Math.abs(b)), '$what: $a vs $b');

  static function output(document:Document):Shape return document.outputFeature().currentShape();

  /** Plate 60..386 x -255..255 x 897..917 with a 55..140 x -150..150 window and four vertical Ø9 holes. */
  static function plateVolume(halfWidth:Float):Float {
    var plate = (386 - 60) * (2 * halfWidth) * 20.0;
    var window = (140 - 60) * 300 * 20.0; // the window starts at x 55, the plate at 60
    var holes = 4 * Math.PI * 4.5 * 4.5 * 20;
    return plate - window - holes;
  }

  public static function run():Void {
    var document = DocumentBuilder.build(function(builder) {
      var half = builder.length("plate.halfWidth", 255).derivedBy("block bolt rule");
      var negHalf = builder.expression("plate.negHalfWidth", ParameterKind.Length, "mm", "-plate.halfWidth");
      var plate = builder.boxBetween(60, 386, negHalf, half, 897, 917);
      var window = builder.boxBetween(55, 140, -150, 150, 896, 918);
      var holes:Array<Feature> = [];
      for (x in [200.0, 300.0]) for (y in [-200.0, 200.0])
        holes.push(builder.cylinderAlong(new Vector(x, y, 0), Vector.Z(), 4.5, 896, 918));
      var tools:Array<Feature> = [window];
      for (hole in holes) tools.push(hole);
      var cut = builder.cutAll(plate, tools, ["front window", "h1", "h2", "h3", "h4"]);
      builder.output(cut);
    });
    near(output(document).volume(), plateVolume(255), "plate with window and holes");

    // The box from bounds matches the immediate API's box moved to its corner (what HobberPartGeometry.box does today).
    var raw = Part.box(326, 510, 20), moved = raw.translated(new Vector((60 + 386) / 2, 0, 897));
    var boundsA = moved.shape.bounds(), single = DocumentBuilder.build(b -> b.output(b.boxBetween(60, 386, -255, 255, 897, 917)));
    var boundsB = output(single).bounds();
    check(Math.abs(boundsA.get_min().get_x() - boundsB.get_min().get_x()) < 1e-9
      && Math.abs(boundsA.get_max().get_z() - boundsB.get_max().get_z()) < 1e-9, "box between matches Part.box + translate");
    near(output(single).volume(), moved.volume(), "same volume as the immediate box");
    raw.close();
    moved.close();
    single.close();

    // The derived width drives the plate through its expression.
    document.parameter("plate.halfWidth").set(260);
    document.recompute();
    near(output(document).volume(), plateVolume(260), "after halfWidth = 260");

    // An axis that is not a coordinate axis: a Ø10 bore 30 long along (1, 1, 0).
    var bore = DocumentBuilder.build(b -> b.output(b.cylinderAlong(new Vector(), new Vector(1, 1, 0), 5, 10, 40)));
    near(output(bore).volume(), Math.PI * 25 * 30, "inclined cylinder volume");
    var centre = output(bore).massProperties().centerOfMass, mid = 25 / Math.sqrt(2);
    check(Math.abs(centre.x - mid) < 1e-6 && Math.abs(centre.y - mid) < 1e-6 && Math.abs(centre.z) < 1e-6, 'bore centre $centre');
    bore.close();

    // Union of overlapping boxes in one step.
    var union = DocumentBuilder.build(function(b) {
      b.output(b.unionAll([b.boxBetween(0, 10, 0, 10, 0, 10), b.boxBetween(5, 15, 0, 10, 0, 10), b.boxBetween(0, 10, 5, 15, 0, 10)]));
    });
    // A 10x10, B adds x 10..15, C adds y 10..15; B and C overlap only inside A: 200 mm2 x 10.
    near(output(union).volume(), 2000, "union of three overlapping boxes");
    var unionText = DocumentCodec.encode(union);
    union.close();
    var unionBack = DocumentCodec.decode(unionText);
    check(DocumentCodec.encode(unionBack) == unionText, "union saves identically after restore");
    unionBack.close();

    var text = DocumentCodec.encode(document);
    document.close();
    var restored = DocumentCodec.decode(text);
    check(DocumentCodec.encode(restored) == text, "plate saves identically after restore");
    near(output(restored).volume(), plateVolume(260), "restored plate");
    restored.parameter("plate.halfWidth").set(250);
    restored.recompute();
    near(output(restored).volume(), plateVolume(250), "restored binding still drives the plate");
    restored.close();
    Sys.println("PlacedPrimitivesSmoke: boxes from bounds, axis cylinders, cut-many and union-all pass");
  }

  public static function main():Void run();
}

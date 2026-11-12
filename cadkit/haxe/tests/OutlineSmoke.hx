import CadKit;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.parametric.recording.Outline;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.RevolveFeature;

/** Named outline edges: every swept face is named after its edge, whatever other edges the outline has. */
class OutlineSmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "outline: " + message;
  }

  static function faces(build:DocumentBuilder->Feature):{names:Array<String>, swept:Feature, volume:Float} {
    var swept:Null<Feature> = null;
    var document = DocumentBuilder.build(b -> {
      swept = build(b);
      b.output(swept);
    });
    var shape = document.result();
    var result = {names: shape.elementNames(CadKit.ShapeKind.Face), swept: (cast swept : Feature), volume: shape.volume()};
    var status = statusOf(sketchOf(result.swept));
    check(status == "fully-constrained", "an outline's sketch is fully defined: " + status);
    document.close();
    return result;
  }

  static function statusOf(sketch:ConstrainedSketchFeature):String {
    var solved = sketch.solvedSketch();
    return solved == null ? "unsolved" : solved.diagnostic.status;
  }

  static function sketchOf(swept:Feature):ConstrainedSketchFeature {
    var source = Std.isOfType(swept, RevolveFeature) ? (cast swept : RevolveFeature).source : (cast swept : ExtrudeFeature).source;
    return cast source;
  }

  /** Editing one dimension of the shaft's sketch reshapes it, and every face keeps its name. */
  static function edited():Void {
    var swept:Null<Feature> = null;
    var document = DocumentBuilder.build(b -> {
      swept = shaft(b, true);
      b.output(swept);
    });
    var turned:Feature = cast swept, before = document.result().volume();
    var sketch = sketchOf(turned);
    sketch.dimension("journal.x").set(3.5);
    sketch.dimension("shoulder.y").set(12);
    sketch.dimension("root.radius").set(0.8);
    document.recompute();
    var names = document.result().elementNames(CadKit.ShapeKind.Face), tag = 'f${turned.id.toInt()}:';
    for (edge in ["input-end", "journal", "root", "shoulder", "lead", "groove", "body", "output-end"])
      check(names.indexOf(tag + DocumentBuilder.sweptFace(turned, edge)) >= 0, 'edited shaft keeps its $edge face: $names');
    // The journal is now radius 3.5 over 12 instead of 4 over 10 and the larger step 2 shorter; the bigger root round adds a
    // little.
    var expected = before + Math.PI * (3.5 * 3.5 * 12 - 16 * 10) - Math.PI * 36 * 2;
    check(Math.abs(document.result().volume() - expected) < 4, "edited shaft volume " + document.result().volume() + " / " + expected);
    check(statusOf(sketch) == "fully-constrained", "the edited sketch stays fully defined");
    document.close();
  }

  /** A stepped shaft's half-section: a journal, a shoulder with a root round, and an optional groove in the larger step. */
  static function shaft(b:DocumentBuilder, groove:Bool):Feature {
    var outline = new Outline(0, 0).to("input-end", 4, 0).to("journal", 4, 10).round("root", 0.5).to("shoulder", 6, 10);
    if (groove) outline.to("lead", 6, 14).to("groove-start", 5, 14).to("groove", 5, 15).to("groove-end", 6, 15);
    return outline.to("body", 6, 30).to("output-end", 0, 30).close("axis").turned(b);
  }

  /** Whether the result names a face that `swept` made from `edge`; a result's names carry the sweep's own tag. */
  static function named(result:{names:Array<String>, swept:Feature, volume:Float}, edge:String):Bool
    return result.names.indexOf('f${result.swept.id.toInt()}:' + DocumentBuilder.sweptFace(result.swept, edge)) >= 0;

  public static function run():Void {
    var plain = faces(b -> shaft(b, false)), grooved = faces(b -> shaft(b, true));
    for (edge in ["input-end", "journal", "root", "shoulder", "body", "output-end"]) {
      check(named(plain, edge), 'plain shaft names its $edge face: ${plain.names}');
      check(named(grooved, edge), 'grooved shaft names its $edge face');
    }
    check(DocumentBuilder.sweptFace(plain.swept, "journal") == DocumentBuilder.sweptFace(grooved.swept, "journal"),
      "a groove elsewhere leaves the journal's name alone");
    check(named(grooved, "groove"), "the groove floor is named");
    // The root round takes material back into the corner: a quarter-circle fillet of 0.5 swept at radius about 4.4.
    check(plain.volume > Math.PI * (16 * 10 + 36 * 20) && plain.volume < Math.PI * (16 * 10 + 36 * 20) + 2,
      "rounded root adds a small ring of material: " + plain.volume);
    check(grooved.volume < plain.volume, "the groove removes material");

    var plate = faces(b -> new Outline(0, 0).to("bottom", 20, 0).round("corner", 3).to("right", 20, 10).to("top", 0, 10)
      .close("left").extruded(b, Plane.XY(), 5));
    for (edge in ["bottom", "corner", "right", "top", "left"])
      check(named(plate, edge), 'extruded plate names its $edge face: ${plate.names}');
    check(Math.abs(plate.volume - 5 * (200 - (9 - Math.PI * 9 / 4))) < 1e-6, "rounded plate volume " + plate.volume);

    var rejected = 0;
    for (attempt in [
      () -> new Outline(0, 0).to("a", 1, 0).to("a", 1, 1).close("b").edges(),
      () -> new Outline(0, 0).to("a b", 1, 0).to("c", 1, 1).close("d").edges(),
      () -> new Outline(0, 0).to("a", 1, 0).round("r", 2).to("c", 1, 1).close("d").edges()
    ]) try {
      attempt();
    } catch (_:Dynamic) rejected++;
    edited();
    check(rejected == 3, "repeated names, escaped characters and oversized rounds are refused");
  }
}

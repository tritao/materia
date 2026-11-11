import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetFeature;

/** Edge flanges on edges chosen by name (wall and outline identity), as recorded code chooses them: same sheet as a face pick, saved by name. */
class SheetNamedEdgeSmoke {
  static var checks = 0;

  static function check(value:Bool, label:String):Void {
    checks++;
    if (!value) throw "SheetNamedEdgeSmoke: " + label;
  }

  static function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-6 * Math.max(1, Math.abs(b)), '$label: $a vs $b');

  static function outline():Array<Vector> return [new Vector(0, 0), new Vector(100, 0), new Vector(100, 60), new Vector(0, 60)];

  static function pick(feature:SheetFeature, wall:Int, identity:String):Shape {
    var graph = feature.sheet(), shape = feature.currentShape();
    for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
      var face = shape.subshape(CadKit.ShapeKind.Face, i);
      try {
        var selected = SheetEdgeSelection.identify(graph, face);
        if (selected.wall == wall && graph.wall(wall).outline.identity(selected.edge.loop, selected.edge.segment) == identity) return face;
      } catch (_:Dynamic) {}
      face.close();
    }
    throw 'no side face $wall:$identity';
  }

  public static function run():Int {
    checks = 0;
    // By name: up along e0, down along e2 with end gaps, then a return flange on the first flange's tip.
    var b = new DocumentBuilder();
    var base = b.sheetBase(b.polygonSketch(outline()), SheetDemoParts.materialA(), b.length("thickness", 2));
    var up = b.sheetFlange(base, [{wall: 1, identity: "sketch:e0"}], b.length("up", 10), b.angle("upAngle", Math.PI / 2), b.length("r", 2));
    var down = b.sheetFlange(up, [{wall: 1, identity: "sketch:e2"}], 20, -Math.PI / 2, 2, Leg, BendOutside, 0, 30, 40);
    var tip = b.sheetFlange(down, [{wall: up.wallIds()[0], identity: "flange:tip"}], 5, Math.PI / 2, 2);
    b.output(tip);
    var named = b.finish();
    var shape = tip.currentShape();
    check(CadKit.shapeValidChecked(shape.borrowHandle()) != 0, "named flanges fold to a valid solid");
    var area = FlatPattern.of(tip.sheet()).area();

    // The same sheet with the first flange picked as a face.
    var p = new DocumentBuilder();
    var pickedBase = p.sheetBase(p.polygonSketch(outline()), SheetDemoParts.materialA(), p.length("thickness", 2));
    pickedBase.document.recompute();
    var face = pick(pickedBase, 1, "sketch:e0");
    var picked = p.sheetEdgeFlange(pickedBase, [face], 10, Math.PI / 2, 2);
    face.close();
    var pickedDown = p.sheetFlange(picked, [{wall: 1, identity: "sketch:e2"}], 20, -Math.PI / 2, 2, Leg, BendOutside, 0, 30, 40);
    var pickedTip = p.sheetFlange(pickedDown, [{wall: picked.wallIds()[0], identity: "flange:tip"}], 5, Math.PI / 2, 2);
    p.output(pickedTip);
    var byFace = p.finish();
    near(pickedTip.currentShape().volume(), shape.volume(), "named and picked edges make the same sheet");
    near(FlatPattern.of(pickedTip.sheet()).area(), area, "same flat pattern");
    byFace.close();

    var text = DocumentCodec.encode(named);
    check(text.indexOf('sketch:e0') >= 0 && text.indexOf('flange:tip') >= 0, "edges saved by name");
    var restored = DocumentCodec.decode(text);
    check(DocumentCodec.encode(restored) == text, "save after restore is identical");
    var again = Std.downcast(restored.featureById(tip.id.toInt()), SheetEdgeFlangeFeature);
    check(again != null && again.namedEdges()[0].identity == "flange:tip", "restored flange keeps its named edge");
    restored.parameter("up").set(15);
    restored.recompute();
    near(again.currentShape().volume(), shape.volume() + 2 * 100 * 5, "editing a named flange's length regrows it");
    restored.close();

    var missing = new DocumentBuilder();
    var missingBase = missing.sheetBase(missing.polygonSketch(outline()), SheetDemoParts.materialA());
    missing.output(missing.sheetFlange(missingBase, [{wall: 1, identity: "sketch:e9"}], 10, Math.PI / 2, 2));
    var failed = false;
    try missing.finish().close() catch (_:Dynamic) failed = true;
    check(failed, "a name the wall does not have is an error");
    named.close();
    Sys.println('SheetNamedEdgeSmoke: $checks checks');
    return checks;
  }

  public static function main():Void run();
}

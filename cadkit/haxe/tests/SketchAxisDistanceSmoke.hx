import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Horizontal and vertical distances: signed offsets between two points that drive, measure and save like any dimension. */
class SketchAxisDistanceSmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "axis distances: " + message;
  }

  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 1e-7, message + ": " + actual + " / " + expected);

  public static function run():Void {
    // The sign says which side: the same sketch solved for offsets to the right and above, and to the left and below.
    for (sign in [1.0, -1.0]) {
      var sketch = new ConstrainedSketch();
      sketch.addPoint(new SketchPoint("a", 0, 0)).addPoint(new SketchPoint("b", sign * 3, sign * 2));
      sketch.addConstraint(SketchConstraint.fixed("origin", "a"))
        .addConstraint(SketchConstraint.horizontalDistance("across", "a", "b", sign * 7))
        .addConstraint(SketchConstraint.verticalDistance("up", "a", "b", sign * 4));
      var solved = sketch.solve();
      near(solved.x("b"), sign * 7, "horizontal offset keeps its sign");
      near(solved.y("b"), sign * 4, "vertical offset keeps its sign");
      check(solved.diagnostic.status == "fully-constrained", "two offsets from a fixed point define it: " + solved.diagnostic.status);
    }

    // A rectangle dimensioned from one corner, as a part drawing is: the far corner moves when a dimension changes.
    var rectangle = new ConstrainedSketch();
    for (p in [{id: "p0", x: 0.0, y: 0.0}, {id: "p1", x: 9.0, y: 0.0}, {id: "p2", x: 9.0, y: 4.0}, {id: "p3", x: 0.0, y: 4.0}])
      rectangle.addPoint(new SketchPoint(p.id, p.x, p.y));
    for (i in 0...4) rectangle.addEntity(SketchEntity.line('e$i', 'p$i', 'p${(i + 1) % 4}'));
    rectangle.addConstraint(SketchConstraint.fixed("origin", "p0"))
      .addConstraint(SketchConstraint.horizontal("bottom", "e0"))
      .addConstraint(SketchConstraint.vertical("right", "e1"))
      .addConstraint(SketchConstraint.horizontal("top", "e2"))
      .addConstraint(SketchConstraint.vertical("left", "e3"))
      .addConstraint(SketchConstraint.horizontalDistance("width", "p0", "p1", 10))
      .addConstraint(SketchConstraint.verticalDistance("height", "p0", "p3", 5))
      .addConstraint(SketchConstraint.horizontalDistance("span", "p3", "p2", 0).asReference());
    var solved = rectangle.solve();
    near(solved.x("p2"), 10, "the far corner follows the width");
    near(solved.y("p2"), 5, "the far corner follows the height");
    near(solved.measured("span"), 10, "a reference horizontal distance measures");
    check(solved.diagnostic.status == "fully-constrained", "the dimensioned rectangle is fully defined");

    // A signed dimension may be negative; a plain distance may not.
    var rejected = false;
    var negative = new ConstrainedSketch();
    negative.addPoint(new SketchPoint("a", 0, 0)).addPoint(new SketchPoint("b", 1, 1));
    negative.addConstraint(SketchConstraint.fixed("origin", "a")).addConstraint(SketchConstraint.distance("d", "a", "b", -2));
    try negative.solve() catch (_:Dynamic) rejected = true;
    check(rejected, "a negative plain distance is refused");

    // As a document feature, the dimension is a parameter that drives the sketch and survives a save.
    var feature:Null<ConstrainedSketchFeature> = null;
    var document = DocumentBuilder.build(b -> {
      feature = b.constrainedSketch(rectangle, new Map());
      b.output(b.extrude(feature, 1));
    });
    var sketchFeature:ConstrainedSketchFeature = cast feature;
    sketchFeature.dimension("width").set(-12);
    document.recompute();
    near(Math.abs(document.result().volume()), 60, "a negative width draws the rectangle to the left");
    var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
    loaded.recompute();
    near(loaded.result().volume(), document.result().volume(), "the edited signed dimension survives a save");
    loaded.close();
    document.close();
  }
}

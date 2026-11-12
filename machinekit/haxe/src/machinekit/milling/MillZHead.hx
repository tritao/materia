package machinekit.milling;

import cadkit.modeling.Plane;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Ribbed fabricated steel spindle head. Origin on rear mounting face at bottom;
 * projects toward -Y. Plate/rib thicknesses are explicit design assumptions.
 */
class MillZHead extends MachineComponent {
  public static inline var COUNTERBORE_DEPTH:Float = 6.5;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [
    P.length("spindleDiameter", 70),
    P.flag("bt30Mounts", false),
    P.length("width", 180),
    P.length("reach", 204),
    P.length("height", 130),
    P.length("noseWidth", 120),
    P.length("rearWidth", 180),
    P.length("rootExtension", 0),
    P.length("rearPlate", 12),
    P.length("boltX", 65),
    P.length("boltZ", 42),
    P.length("spindleReach", 159),
    P.length("plate", 14),
    P.length("wall", 12)
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.milling.z-head", parameters(), v -> new MillZHead(v))
      .numbered(_ -> "MILL-Z-RIBBED-HEAD"));
  }
  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values), r = v.number("spindleDiameter") / 2;
    if (v.number("width") <= 2 * (r + v.number("wall"))
      || v.number("height") <= 3 * v.number("plate") + 8 || v.number("spindleReach") <= r + v.number("wall")
      || v.number("reach") - v.number("spindleReach") <= r || v.number("wall") <= 0
      || v.number("plate") <= 0) throw "Z head must surround its spindle bore and retain connected ribs";
    if (v.number("noseWidth") <= 2 * (r + v.number("wall")) || v.number("noseWidth") > v.number("width")
      || v.number("rearWidth") < v.number("width") || v.number("rearPlate") < 10 || v.number("rootExtension") < 0
      || v.number("rootExtension") > v.number("height") / 2 || v.number("boltX") + 7 > v.number("rearWidth") / 2
      || 2 * (v.number("boltZ") + 7) > v.number("height")) throw "Head mounting flange cannot retain its bolt pattern";
    super(
      "MILL-Z-RIBBED-HEAD",
      "Ribbed fabricated spindle head",
      v.get("material") == null ? "steel" : v.token("material"),
      false,
      v
    );
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("spindle", Axis, Solids.axial(0, -v.number("spindleReach"), 0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  /** The plan profile `points` (x, y) extruded up from z0 to z1. */
  static function prism(b:DocumentBuilder, points:Array<Array<Float>>, z0:Float, z1:Float):Feature
    return b.extrude(b.polygon([for (p in points) new Vector(p[0], p[1], 0)], new Plane(new Vector(0, 0, z0), Vector.X(), Vector.Z())), z1 - z0);

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var w = parameterNumber("width"), d = parameterNumber("reach"), h = parameterNumber("height");
    var plate = parameterNumber("plate"), wall = parameterNumber("wall"), spindle = parameterNumber("spindleReach");
    var collar = 2 * (parameterNumber("spindleDiameter") / 2 + 10), collarRear = -spindle + collar / 2;
    var rearPlate = parameterNumber("rearPlate"), nose = parameterNumber("noseWidth"), extension = parameterNumber("rootExtension");
    var parts:Array<Feature> = [b.label(b.boxBetween(-w / 2, w / 2, -spindle - collar / 2, -spindle + collar / 2, 0, h), "Spindle collar")];
    for (z in [0.0, h - plate]) parts.push(b.label(b.boxBetween(-w / 2, w / 2, -d, 0, z, z + plate), z == 0 ? "Bottom plate" : "Top plate"));
    for (side in [-1, 1]) parts.push(b.label(prism(b, [
      [side * w / 2, 0],
      [side * nose / 2, collarRear],
      [side * (nose / 2 - wall), collarRear],
      [side * (w / 2 - wall), 0]
    ], 0, h), side < 0 ? "Side wall left" : "Side wall right"));
    // Two cross webs connect the side walls, top and bottom into one body.
    for (k => fraction in [1.0 / 3, 2.0 / 3]) {
      var y = collarRear * fraction;
      parts.push(b.label(b.boxBetween(-w / 2, w / 2, y - wall / 2, y + wall / 2, 0, h), 'Cross web ${k + 1}'));
    }
    var outline = b.label(prism(b, [[-w / 2, 0], [w / 2, 0], [nose / 2, collarRear], [nose / 2, -d], [-nose / 2, -d], [-nose / 2, collarRear]], 0, h), "Taper outline");
    var tapered = b.label(b.intersect(b.unionAll(parts), outline), "Tapered head");
    var root:Array<Feature> = [tapered, b.label(b.boxBetween(-parameterNumber("rearWidth") / 2, parameterNumber("rearWidth") / 2, -rearPlate, 0, -extension, h + extension), "Rear flange")];
    if (extension > 0) for (side in [-1, 1]) for (upper in [false, true]) {
      var z = upper ? h : 0, outer = upper ? h + extension : -extension;
      var x = side * (upper && values().boolean("bt30Mounts") ? 50 : w / 2 - 2 * wall);
      var profile = b.polygon([new Vector(0, z, 0), new Vector(collarRear / 2, z, 0), new Vector(0, outer, 0)], new Plane(new Vector(x - wall / 2, 0, 0), Vector.Y(), Vector.X()));
      root.push(b.label(b.extrude(profile, wall), 'Root rib ${side < 0 ? "left" : "right"} ${upper ? "upper" : "lower"}'));
    }
    var body = b.label(b.unionAll(root), "Head");
    var tools:Array<Feature> = [b.cylinderAlong(new Vector(0, -spindle, 0), Vector.Z(), parameterNumber("spindleDiameter") / 2, -1, h + 1)];
    var labels = ["spindle bore"];
    // Two rows of side windows leave top/bottom chords, cross webs and a middle rib.
    for (row in 0...2) for (cell in 0...3) {
      var rear = cell == 0 ? -rearPlate - 2 : collarRear * cell / 3 - wall / 2 - 2;
      var front = collarRear * (cell + 1) / 3 + wall / 2 + 2, z = plate + row * (h - plate) / 2;
      if (rear > front) {
        tools.push(b.boxBetween(-w / 2 - 1, w / 2 + 1, front, rear, z, z + (h - 3 * plate) / 2));
        labels.push('window ${row == 0 ? "lower" : "upper"} ${cell + 1}');
      }
    }
    // Front-access head bolts pass only through the rear flange. Counterbores seat M6 heads.
    for (x in [-parameterNumber("boltX"), parameterNumber("boltX")]) for (z in [h / 2 - parameterNumber("boltZ"), h / 2 + parameterNumber("boltZ")]) {
      var at = (x < 0 ? "left " : "right ") + (z < h / 2 ? "low" : "high");
      tools.push(b.cylinderAlong(new Vector(x, 0, z), Vector.Y(), 3.3, -rearPlate - 1, 1));
      labels.push('head bolt $at');
      tools.push(b.cylinderAlong(new Vector(x, 0, z), Vector.Y(), 5.5, -rearPlate - 1, -rearPlate + COUNTERBORE_DEPTH));
      labels.push('head bolt counterbore $at');
    }
    if (values().boolean("bt30Mounts")) {
      for (x in [-MillSpindleMotorLayout.FOOT_BOLT_X, MillSpindleMotorLayout.FOOT_BOLT_X]) for (y in [MillSpindleMotorLayout.FOOT_BOLT_FRONT, MillSpindleMotorLayout.FOOT_BOLT_REAR]) {
        tools.push(b.cylinderAlong(new Vector(x, -spindle + y, 0), Vector.Z(), 2.5, h - 12, h + 1));
        labels.push('carrier tap ${x < 0 ? "left" : "right"} ${y == MillSpindleMotorLayout.FOOT_BOLT_FRONT ? "front" : "rear"}');
      }
      for (i in 0...6) {
        var a = i * Math.PI / 3;
        tools.push(b.cylinderAlong(new Vector(53.5 * Math.cos(a), -spindle + 53.5 * Math.sin(a), 0), Vector.Z(), 2.5, -1, 12));
        labels.push('cartridge tap ${i + 1}');
      }
    }
    return b.label(b.cutAll(body, tools, labels), "Bore, windows and bolt holes");
  }
}

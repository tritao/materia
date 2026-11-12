package machinekit.milling;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** T-slot table with integral longitudinal ribs and a recessed X nut bracket seat.
 * Origin is the underside of the deck; ribs extend down to the guide blocks. */
class MillCrossTable extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.cross-table",
      [
        P.length("depth", 200),
        P.length("ribSpacing", 124),
        P.length("width", 400),
        P.length("thickness", 40),
        P.length("ribHeight", 48),
        P.length("ribWidth", 34),
        P.flag("nutSeat", true)
      ],
      v -> new MillCrossTable(
        v.number("depth"),
        v.number("ribSpacing"),
        v.number("width"),
        v.number("thickness"),
        v.number("ribHeight"),
        v.number("ribWidth"),
        v.boolean("nutSeat")
      )
    ).numbered(_ -> "MILL-CROSS-TABLE"));
  }

  public function new(
    depth:Float = 200,
    ribSpacing:Float = 124,
    width:Float = 400,
    thickness:Float = 40,
    ribHeight:Float = 48,
    ribWidth:Float = 34,
    nutSeat:Bool = true
  ) {
    for (v in [width, thickness,
      ribHeight, ribWidth]) if (!Math.isFinite(v) || v <= 0) throw "Table dimensions must be finite and positive";
    if (width < 400 || thickness < 40 || ribWidth < 34 || ribSpacing + ribWidth > depth - 12
      || (nutSeat && ribSpacing - ribWidth < 86)) throw "Table ribs and slots need retained edge margins";
    if (!Math.isFinite(depth) || !Math.isFinite(ribSpacing)
      || depth < 200 || ribSpacing < 124) throw "Table must retain its slots, drive corridor and rib edge margins";
    super(
      "MILL-CROSS-TABLE",
      "Ribbed T-slot table",
      "cast iron",
      false,
      new ComponentValues().setNumber("depth",
      depth).setNumber("ribSpacing", ribSpacing).setNumber("width", width).setNumber(
        "thickness",
        thickness
      ).setNumber(
        "ribHeight",
        ribHeight
      ).setNumber(
        "ribWidth",
        ribWidth
      ).setBoolean(
        "nutSeat",
        nutSeat
      )
    );
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Face, Solids.axial(0, 0, thickness));
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function fasteningFaces():Array<BoltedFace> return [for (side in [-1, 1]) BoltedFace.drilledToSuit(
    'coverEnd$side', BoltedFace.frameOf({x: side * parameterNumber("width") / 2, y: 0, z: 18},
      {x: -side, y: 0, z: 0}, {x: 0, y: 1, z: 0}), parameterNumber("depth"), 16, 12, 30, CastIron)];
  override public function machinable():Bool return true;
  override public function hasGeometry():Bool return true;

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var width = parameterNumber("width"), depth = parameterNumber("depth"), thickness = parameterNumber("thickness");
    var ribHeight = parameterNumber("ribHeight"), ribWidth = parameterNumber("ribWidth"), x = width / 2;
    var parts:Array<Feature> = [b.label(b.boxBetween(-x, x, -depth / 2, depth / 2, 0, thickness), "Deck")];
    for (side in [-1, 1]) {
      var y = side * parameterNumber("ribSpacing") / 2;
      parts.push(b.label(b.boxBetween(-x, x, y - ribWidth / 2, y + ribWidth / 2, -ribHeight, 0), side < 0 ? "Front rib" : "Rear rib"));
    }
    var body = b.label(b.unionAll(parts), "Table casting");
    var tools:Array<Feature> = [], labels:Array<String> = [];
    var seat = values().boolean("nutSeat");
    if (seat) {
      tools.push(b.boxBetween(-1, 41, -43, 43, -1, 12));
      labels.push("nut bracket seat");
    }
    var slots = Std.int((depth / 2 - 20) / 40);
    for (slot in -slots...slots + 1) {
      var y = slot * 40.0;
      tools.push(b.boxBetween(-x - 1, x + 1, y - 5, y + 5, thickness - 8, thickness + 1));
      labels.push('T-slot ${slot + slots + 1} throat');
      tools.push(b.boxBetween(-x - 1, x + 1, y - 9, y + 9, thickness - 14, thickness - 8));
      labels.push('T-slot ${slot + slots + 1} head');
    }
    if (seat) for (y in [-25.0, 25.0]) {
      var at = y < 0 ? "front" : "rear";
      tools.push(b.cylinderAlong(new Vector(20, y, 0), Vector.Z(), 3.3, 11, 41));
      labels.push('nut bracket screw $at');
      tools.push(b.cylinderAlong(new Vector(20, y, 0), Vector.Z(), 5.5, 33, 41));
      labels.push('nut bracket counterbore $at');
    }
    return b.label(b.cutAll(body, tools, labels), seat ? "T-slots and nut bracket seat" : "T-slots");
  }
}

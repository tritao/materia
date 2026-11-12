package machinekit.milling;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.BallNut;

/** Welded steel carriage plate and central nut shelf. Origin: rear plate face,
 * centre X, bottom Z; +Y goes back toward the recessed screw. Nominal holes.
 */
class MillZCarriage extends MachineComponent {
  public static inline var GUSSET_HEIGHT:Float = 20;
  public static inline var SHELF_MARGIN:Float = 6;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [
    P.choice("nutModel", ["generic1605", "SFU2005"], "generic1605"),
    P.length("width", 234),
    P.length("height", 160),
    P.length("thickness", 16),
    P.length("nutReach", 53),
    P.length("nutCentreZ", 80),
    P.length("nutPlate", 10),
    P.length("headBoltX", 65),
    P.length("headBoltZ", 42),
    P.length("railSpacing", 180),
    P.length("blockSpan", 32.7),
    P.text("profile", "HGH15CA")
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.z-carriage",
      parameters(),
      v -> new MillZCarriage(v)
    ).numbered(_ -> "MILL-Z-CARRIAGE"));
  }
  public static function selectedNut(model:String):machinekit.motion.LeadScrewNut return model
    == "SFU2005" ? new machinekit.motion.SfuNut(
      model,
      16
    ) : new BallNut();
  static function guide(profile:String):machinekit
    .motion.LinearRailBlock return machinekit.motion.LinearRailBlock.metric(profile);

  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values), block = guide(v.token("profile")), nut = selectedNut(v.token("nutModel"));
    var width = v.number("width"), height = v.number("height"), t = v.number("thickness");
    if (width <= v.number("railSpacing") + block.spec.blockWidth || height <= 2 * v.number("blockSpan")
      + block.spec.blockLength || t <= 0 || v.number("nutReach") <= nut.flangeDiameter / 2 || v.number("nutPlate") <= 0
      || v.number("nutCentreZ") + nut.bodyLength / 2 - nut.flangeOffset - nut.flangeThickness
      - v.number("nutPlate") - GUSSET_HEIGHT <= 0) throw "Z carriage leaves insufficient plate or nut-shelf material";
    super(
      "MILL-Z-CARRIAGE",
      "Welded Z carriage with central ball-nut shelf",
      v.get("material") == null ? "steel" : v.token("material"),
      false,
      v
    );
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var w = parameterNumber("width"), h = parameterNumber("height"), t = parameterNumber("thickness");
    var nut = selectedNut(parameterToken("nutModel")),
      reach = parameterNumber("nutReach"), shelf = parameterNumber("nutPlate");
    var face = parameterNumber("nutCentreZ") + nut.bodyLength / 2 - nut.flangeOffset - nut.flangeThickness;
    var shelfWidth = nut.flangeDiameter + 2 * SHELF_MARGIN, depth = reach + shelfWidth / 2, gusset = Math.min(reach / 2, 25);
    var parts:Array<Feature> = [
      b.label(b.boxBetween(-w / 2, w / 2, -t, 0, 0, h), "Plate"),
      b.label(b.boxBetween(-shelfWidth / 2, shelfWidth / 2, 0, depth, face - shelf, face), "Nut shelf")
    ];
    for (side in [-1, 1]) {
      var x = side * (shelfWidth / 2 - 3);
      parts.push(b.label(b.boxBetween(x - 3, x + 3, 0, gusset, face - shelf - GUSSET_HEIGHT, face - shelf), side < 0 ? "Gusset left" : "Gusset right"));
    }
    var body = b.label(b.unionAll(parts), "Carriage");
    var up = Vector.Z(), back = Vector.Y();
    var tools:Array<Feature> = [b.cylinderAlong(new Vector(0, reach, 0), up, nut.bodyDiameter / 2 + 0.2, face - shelf - 1, face + 1)];
    var labels = ["nut bore"], i = 0;
    for (point in nut.boltPattern()) {
      tools.push(b.cylinderAlong(new Vector(point.x, reach + point.y, 0), up, nut.mountScrew == "M6" ? 3.35 : 2.75, face - shelf - 1, face + 1));
      labels.push('nut screw ${++i}');
    }
    var block = guide(parameterToken("profile")), across = block.spec.blockHoleSpacingAcross / 2, along = block.spec.blockHoleSpacingAlong / 2;
    var blockTap = block.spec.blockMountThread == "M6" ? 3.3 : 2.25, span = parameterNumber("blockSpan");
    for (side in [-1, 1]) for (s in [-1, 1]) for (x in [-across, across]) for (z in [-along, along]) {
      var px = side * parameterNumber("railSpacing") / 2 + x, pz = parameterNumber("nutCentreZ") + s * span + z;
      tools.push(b.cylinderAlong(new Vector(px, 0, pz), back, blockTap, -t - 1, 1));
      labels.push('block screw ${side < 0 ? "left" : "right"} ${s < 0 ? "lower" : "upper"} ${x < 0 ? "inner" : "outer"} ${z < 0 ? "low" : "high"}');
    }
    // Blind nominal M6 taps match the head flange; retain the rear face of the carriage.
    for (x in [-parameterNumber("headBoltX"), parameterNumber("headBoltX")]) for (z in [h / 2 - parameterNumber("headBoltZ"), h / 2 + parameterNumber("headBoltZ")]) {
      tools.push(b.cylinderAlong(new Vector(x, 0, z), back, 2.5, -t - 1, -t + 12));
      labels.push('head tap ${x < 0 ? "left" : "right"} ${z < h / 2 ? "low" : "high"}');
    }
    return b.label(b.cutAll(body, tools, labels), "Nut bore, nut screws, block screws and head taps");
  }
}

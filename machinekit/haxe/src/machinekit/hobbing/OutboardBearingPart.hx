package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Made coaxial support parts in the housing datum. The sleeve carries both
 * outer races; its gland closes their matched ground faces. The inner spacer
 * lets a standard-sized retaining nut clear the fixed gland. */
class OutboardBearingPart extends MachineComponent {

  public static function capHoles():Array<machinekit.component.BoltedFace.BoltHole> return [
    for (x in [-23.0, 23.0]) for (y in [-12.0, 12.0]) {x: x, y: y}
  ];
  public static function capFrame(into:Float):AssemblyFrame return BoltedFace.frameOf({x: 0.0, y: 0.0, z: OutboardSupport.BODY_LENGTH},
    {x: 0, y: 0, z: into}, {x: 1, y: 0, z: 0});

  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [P.choice("role", ["sleeve", "gland", "cap", "innerSpacer"], "sleeve")];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hobbing.outboard-bearing-part", parameters(), v -> new OutboardBearingPart(v), false, false, null, false)
      .numbered(v -> "HOB-OUTBOARD-" + v.token("role").toUpperCase()).procured(v -> Make(v.token("role") == "cap" ? "CNC milling" : "turning and grinding"))
      .requiring(v -> routeFor(v.token("role")).requirements()));
  }
  /** The cap is milled; the sleeve, gland and inner spacer are turned and ground to their working faces. */
  static function routeFor(role:String):ManufacturingRoute {
    var faces:Array<machinekit.component.ManufacturingRoute.MachinedFace> = switch role {
      case "sleeve": [
        {name: "bearing bore D32; D40 g5 sliding register, coaxial; front D21 labyrinth", tolerance: 0.002, roughnessRa: 0.4},
        {name: "outer-ring shoulder and M36x1 gland seat", tolerance: 0.002, roughnessRa: 0.4}];
      case "gland": [{name: "outer-ring clamping face, square to M36x1", tolerance: 0.002, roughnessRa: 0.4}];
      case "innerSpacer": [{name: "inner-ring spacer ends, parallel", tolerance: 0.002, roughnessRa: 0.4}];
      case "cap": [{name: "axial float stop face and D40 register", tolerance: 0.01, roughnessRa: 1.6}];
      default: throw "unknown outboard bearing part";
    };
    return role == "cap" ? ManufacturingRoute.milled(2, "fine", faces, 60) : ManufacturingRoute.turned(2, "tight", faces);
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides), role = v.token("role");
    super("HOB-OUTBOARD-" + role.toUpperCase(), "Outboard bearing " + role, "steel", false, v);
    addFacet(routeFor(role));
    addFacet(machinekit.component.EngineeringMetadata.assumed("Turned and ground to the stated faces; sizes from the DB7002 bearing, fits assumed"));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return values().token("role") == "cap";
  override public function fastensByForm():Bool return ["gland", "innerSpacer"].indexOf(values().token("role")) >= 0;
  override public function fasteningFaces():Array<BoltedFace> return values().token("role") == "cap" ? [
    BoltedFace.clamped("housing", OutboardBearingPart.capFrame(1), OutboardBearingPart.capHoles(), 4.5, OutboardSupport.CAP_THICKNESS, "M4", null, false)
  ] : [];
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return switch values().token("role") {
    case "sleeve": Solids.cut(Solids.named(Part.cylinderSpan(20, OutboardSupport.SLEEVE_FROM, OutboardSupport.SLEEVE_TO), "sleeve"), [
      Solids.named(Part.cylinderSpan(10.5, OutboardSupport.SLEEVE_FROM - 1, OutboardSupport.PAIR_FROM), "shaft-bore"),
      Solids.named(Part.cylinderSpan(16, OutboardSupport.PAIR_FROM, OutboardSupport.PAIR_TO), "pair-seat"),
      Solids.named(Part.cylinderSpan(18, OutboardSupport.GLAND_FROM, OutboardSupport.GLAND_TO + 1), "gland-seat")
    ]);
    case "gland":
      var tools = [Solids.named(Part.cylinderSpan(14, OutboardSupport.GLAND_FROM - 1, OutboardSupport.GLAND_TO + 1), "bore")];
      // Blind pin-spanner holes, not fastener holes. M36x1 represented nominally.
      for (k => x in [-16.0, 16.0]) tools.push(Solids.named(Part.cylinderSpan(1, OutboardSupport.GLAND_TO - 2, OutboardSupport.GLAND_TO + 0.1, x, 0), "spanner-hole" + (k + 1)));
      Solids.cut(Solids.named(Part.cylinderSpan(18, OutboardSupport.GLAND_FROM, OutboardSupport.GLAND_TO), "gland"), tools);
    case "innerSpacer": Solids.cut(Solids.named(Part.cylinderSpan(OutboardSupport.SPACER_DIAMETER / 2, OutboardSupport.PAIR_TO, OutboardSupport.PAIR_TO + OutboardSupport.SPACER_LENGTH), "spacer"),
      [Solids.named(Part.cylinderSpan(7.5, OutboardSupport.PAIR_TO - 1, OutboardSupport.PAIR_TO + OutboardSupport.SPACER_LENGTH + 1), "bore")]);
    case "cap": machined(Solids.cut(Solids.union([
      Solids.named(Part.box(60, 60, OutboardSupport.CAP_THICKNESS).translated(new Vector(0, 0, OutboardSupport.BODY_LENGTH)), "cap"),
      Solids.named(Part.cylinderSpan(20, OutboardSupport.REAR_STOP, OutboardSupport.BODY_LENGTH), "spigot")
    ]), [Solids.named(Part.cylinderSpan(OutboardSupport.CAP_BORE / 2, OutboardSupport.REAR_STOP - 1, OutboardSupport.BODY_LENGTH + OutboardSupport.CAP_THICKNESS + 1), "bore")]));
    default: throw "unknown outboard bearing part";
  };
}

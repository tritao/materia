package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.hobbing.HobberOutboardLayout as L;

/** Made coaxial support parts in the housing datum. The sleeve carries both
 * outer races; its gland closes their matched ground faces. The inner spacer
 * lets a standard-sized retaining nut clear the fixed gland. */
class OutboardBearingPart extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [P.choice("role", ["sleeve", "gland", "cap", "innerSpacer"], "sleeve")];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hobbing.outboard-bearing-part", parameters(), v -> new OutboardBearingPart(v), false, false, null, false)
      .numbered(v -> "HOB-OUTBOARD-" + v.token("role").toUpperCase()).procured(v -> Make(v.token("role") == "cap" ? "CNC milling" : "turning and grinding")));
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides), role = v.token("role");
    super("HOB-OUTBOARD-" + role.toUpperCase(), "Outboard bearing " + role, "steel", false, v);
    var faces:Array<machinekit.component.ManufacturingRoute.MachinedFace> = switch role {
      case "sleeve": [
        {name: "bearing bore D32; D40 g5 sliding register, coaxial; front D21 labyrinth", tolerance: 0.002, roughnessRa: 0.4},
        {name: "outer-ring shoulder and M36x1 gland seat", tolerance: 0.002, roughnessRa: 0.4}];
      case "gland": [{name: "outer-ring clamping face, square to M36x1", tolerance: 0.002, roughnessRa: 0.4}];
      case "innerSpacer": [{name: "inner-ring spacer ends, parallel", tolerance: 0.002, roughnessRa: 0.4}];
      case "cap": [{name: "axial float stop face and D40 register", tolerance: 0.01, roughnessRa: 1.6}];
      default: throw "unknown outboard bearing part";
    };
    addFacet(role == "cap" ? ManufacturingRoute.milled(2, "fine", faces, 60) : ManufacturingRoute.turned(2, "tight", faces));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return values().token("role") == "cap";
  override public function fastensByForm():Bool return ["gland", "innerSpacer"].indexOf(values().token("role")) >= 0;
  override public function fasteningFaces():Array<BoltedFace> return values().token("role") == "cap" ? [
    BoltedFace.clamped("housing", L.capFrame(1), L.capHoles(), 4.5, L.CAP_THICKNESS, "M4", null, false)
  ] : [];
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return switch values().token("role") {
    case "sleeve": Solids.cut(Part.cylinderSpan(20, L.SLEEVE_FROM, L.SLEEVE_TO), [
      Part.cylinderSpan(10.5, L.SLEEVE_FROM - 1, L.PAIR_FROM),
      Part.cylinderSpan(16, L.PAIR_FROM, L.PAIR_TO),
      Part.cylinderSpan(18, L.GLAND_FROM, L.GLAND_TO + 1)
    ]);
    case "gland":
      var tools = [Part.cylinderSpan(14, L.GLAND_FROM - 1, L.GLAND_TO + 1)];
      // Blind pin-spanner holes, not fastener holes. M36x1 represented nominally.
      for (x in [-16.0, 16.0]) tools.push(Part.cylinderSpan(1, L.GLAND_TO - 2, L.GLAND_TO + 0.1, x, 0));
      Solids.cut(Part.cylinderSpan(18, L.GLAND_FROM, L.GLAND_TO), tools);
    case "innerSpacer": Solids.cut(Part.cylinderSpan(L.SPACER_DIAMETER / 2, L.PAIR_TO, L.PAIR_TO + L.SPACER_LENGTH),
      [Part.cylinderSpan(7.5, L.PAIR_TO - 1, L.PAIR_TO + L.SPACER_LENGTH + 1)]);
    case "cap": machined(Solids.cut(Solids.union([
      Part.box(60, 60, L.CAP_THICKNESS).translated(new Vector(0, 0, L.BODY_LENGTH)),
      Part.cylinderSpan(20, L.REAR_STOP, L.BODY_LENGTH)
    ]), [Part.cylinderSpan(L.CAP_BORE / 2, L.REAR_STOP - 1, L.BODY_LENGTH + L.CAP_THICKNESS + 1)]));
    default: throw "unknown outboard bearing part";
  };
}

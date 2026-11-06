package machinekit.hobbing;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.hobbing.HobberOutboardLayout as L;

/** Made bearing housing. A 40 mm sliding sleeve carries a matched DB pair;
 * end stops permit +/-1 mm thermal float. Side bolts permit indication/shims,
 * then matching dowels locate it on the removable structural carrier. */
class OutboardSupport extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("radialRunout", L.RADIAL_BUDGET, "mm"),
    P.quantity("radialStiffness", 100000000, "N/m")
  ]);
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hobbing.outboard-support", parameters(), v -> new OutboardSupport(v), false, false, null, false)
      .versioned(2).numbered(_ -> "HOB-OUTBOARD-HOUSING-D40-DB7002")
      .procured(_ -> Make("CNC milling and bore finishing")));
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super("HOB-OUTBOARD-HOUSING-D40-DB7002", "Indicated and pinned housing for floating DB7002 sleeve", "steel", false, v);
    addFacet(new EngineeringMetadata(v));
    addFacet(machinekit.motion.PrecisionErrors.fromValues(v, [{name: "radialRunout", unit: "mm"}]));
    addFacet(ManufacturingRoute.milled(2, "tight", [
      {name: "sleeve sliding bore D40 H6, aligned to spindle by indication", tolerance: 0.002, roughnessRa: 0.4},
      {name: "carrier face and pin bores (ream after alignment)", tolerance: 0.002, roughnessRa: 0.8},
      {name: "rear cap face, square to sliding bore", tolerance: 0.01, roughnessRa: 1.6}
    ], L.WIDTH));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
  }
  override public function machinable():Bool return true;
  override public function componentType():Null<ComponentType> return recipeType();
  override public function fasteningFaces():Array<BoltedFace> return [
    BoltedFace.drilledToSuit("carrier", L.housingMountFrame(), 30, 60, 8, 15, BoltedFace.ThreadMaterial.Steel),
    BoltedFace.drilledToSuit("cap", L.capFrame(-1), 60, 60, 10, 30, BoltedFace.ThreadMaterial.Steel)
  ];
  override public function pinnedFaces():Array<PinnedFace> return [L.housingPins()];
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return machined(Solids.cut(
    Part.box(L.WIDTH, L.WIDTH, L.BODY_LENGTH), [
      Part.cylinderSpan(L.SLEEVE_DIAMETER / 2, L.FRONT_STOP, L.BODY_LENGTH + 1),
      Part.cylinderSpan(L.GLAND_BORE / 2, -1, L.FRONT_STOP + 1)
    ]));
}

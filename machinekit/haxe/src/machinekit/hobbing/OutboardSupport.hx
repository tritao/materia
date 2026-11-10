package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ComponentRecipeSupport as P;

/** Made bearing housing. A 40 mm sliding sleeve carries a matched DB pair;
 * end stops permit +/-1 mm thermal float. Side bolts permit indication/shims,
 * then matching dowels locate it on the removable structural carrier. */
class OutboardSupport extends MachineComponent {

  public static inline final WIDTH = 60.0;
  public static inline final BODY_LENGTH = 30.0;
  public static inline final SLEEVE_DIAMETER = 40.0;
  public static inline final SLEEVE_FROM = 3.0;
  public static inline final SLEEVE_TO = 27.0;
  public static inline final FRONT_STOP = 2.0;
  public static inline final REAR_STOP = 28.0;
  public static inline final PAIR_FROM = 6.0;
  public static inline final PAIR_TO = 24.0;
  public static inline final GLAND_FROM = 24.0;
  public static inline final GLAND_TO = 27.0;
  public static inline final GLAND_DIAMETER = 36.0;
  public static inline final GLAND_BORE = 28.0;
  public static inline final CAP_BORE = 30.0;
  public static inline final CAP_THICKNESS = 3.0;
  public static inline final SPACER_LENGTH = 5.0;
  public static inline final SPACER_DIAMETER = 20.0;
  public static inline final NUT_LENGTH = 6.0;
  public static inline final INDICATED_RADIAL = 0.002;
  public static inline final SEAT_RADIAL = 0.001;
  public static inline final BEARING_RADIAL = 0.0015;
  public static inline final THERMAL_RADIAL = 0.0005;
  public static inline final RADIAL_BUDGET = 0.005;
  public static inline final ASSEMBLY_AXIAL = 0.2;
  public static inline final TEMPERATURE_RISE = 30.0;
  public static inline final STEEL_EXPANSION = 12e-6;
  public static function housingMountFrame():AssemblyFrame return BoltedFace.frameOf({x: -30.0, y: 0.0, z: 15.0},
    {x: 1, y: 0, z: 0}, {x: 0, y: 0, z: 1});
  public static function housingPins():PinnedFace return PinnedFace.atPoints("carrier-pins", OutboardSupport.housingMountFrame(),
    [for (y in [-22.0, 22.0]) {x: -30.0, y: y, z: 15.0}], 4, 6, false);

  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("radialRunout", OutboardSupport.RADIAL_BUDGET, "mm"),
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
    ], OutboardSupport.WIDTH));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
  }
  override public function machinable():Bool return true;
  override public function componentType():Null<ComponentType> return recipeType();
  override public function fasteningFaces():Array<BoltedFace> return [
    BoltedFace.drilledToSuit("carrier", OutboardSupport.housingMountFrame(), 30, 60, 8, 15, BoltedFace.ThreadMaterial.Steel),
    BoltedFace.drilledToSuit("cap", OutboardBearingPart.capFrame(-1), 60, 60, 10, 30, BoltedFace.ThreadMaterial.Steel)
  ];
  override public function pinnedFaces():Array<PinnedFace> return [OutboardSupport.housingPins()];
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return machined(Solids.cut(
    Part.box(OutboardSupport.WIDTH, OutboardSupport.WIDTH, OutboardSupport.BODY_LENGTH), [
      Solids.named(Part.cylinderSpan(OutboardSupport.SLEEVE_DIAMETER / 2, OutboardSupport.FRONT_STOP, OutboardSupport.BODY_LENGTH + 1), "sleeve-bore"),
      Solids.named(Part.cylinderSpan(OutboardSupport.GLAND_BORE / 2, -1, OutboardSupport.FRONT_STOP + 1), "gland-bore")
    ]));
}

package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob spindle housing. Reference row is Assumed; dimensions are simplified envelopes. */
class SpindleHousing extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", 80, "mm"),
    P.quantity("bore", 62, "mm"),
    P.quantity("height", 180, "mm")
  ]);
  /**
   * Clamping length (assumed, the maker's drawing is not in hand): the Ø80 land of the housing between its two end registers, in the housing's
   * own z. It stops 10 mm short of the rear face (z 0, the encoder stator stands on it) and 10 mm short of the front face (z 180, the nose and gland flange at
   * 180..186), so a clamp never grips a register or the nose; it must cover the angular-contact pair's outer rings (129..163).
   */
  public static inline final CLAMP_FROM = 10.0;
  public static inline final CLAMP_TO = 170.0;
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.hobbing.spindle-housing",
      parameters(),
      v -> new SpindleHousing(v)
    ).numbered(_ -> "HOB-SPINDLE-HOUSING").procured(_ -> Make("CNC machining")).requiringInputs(["outside", "bore", "height"]);
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues
    > return EngineeringMetadata.assumedCatalog(
      "Hob spindle housing",
      "HOB-SPINDLE-HOUSING",
      parameters(),
      "machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)"
    );
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super("HOB-SPINDLE-HOUSING", "Hob spindle housing (reference envelope)", "steel", false, v);
    if (v.get("material") != null) setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    if (parameterNumber("bore") != 62
      || parameterNumber("height") != 180) throw "cartridge housing requires the 62 mm seats and 180 mm length";
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v, []));

  }
  override public function componentType():Null<ComponentType> return recipeType();

  public function interfaceName():String return "ASSUMED-TF20";
  override public function hasGeometry():Bool return true;
  /** Bores and the eight M4 holes (four each end, on Ø72) for the gland and the encoder stator; threads omitted. */
  override public function geometry(detail:ComponentDetail = Preview):cadkit.modeling.Part {
    var tools = [
      cadkit.modeling.Part.cylinderSpan(26, -0.1, 15),
      cadkit.modeling.Part.cylinderSpan(31, 15, 120),
      // Ø44 shoulder bore: clears the Ø38 locknut and still backs the 7305 outer ring.
      cadkit.modeling.Part.cylinderSpan(22, 120, 129),
      cadkit.modeling.Part.cylinderSpan(31, 129, 180.1)
    ];
    for (i in 0...4) {
      var angle = Math.PI / 4 + i * Math.PI / 2, x = 36 * Math.cos(angle), y = 36 * Math.sin(angle);
      tools.push(cadkit.modeling.Part.cylinderSpan(2, -0.1, 10, x, y));
      tools.push(cadkit.modeling.Part.cylinderSpan(2, 170, 180.1, x, y));
    }
    return Solids.cut(cadkit.modeling.Part.cylinderSpan(parameterNumber("outside") / 2, 0, 180), tools);
  }
}

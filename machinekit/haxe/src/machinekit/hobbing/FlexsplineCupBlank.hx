package machinekit.hobbing;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;

/** One-piece cup blank, open/toothed end at Z=0, closed end above it.
    Tooth band and bore use the existing work datum; dimensions are assumed.
    The bore is the clamping and centring datum: the cup copies the clamped sleeve at ~0.95 um/um and any bore
    or wall error comes back as tooth error after release (workholding/fem/README case 2, loads/README 3).
    Process requirements on the bore, with the reasons for the values:
    - honed after hardening, H5 (58.050/+0.013): the loading clearance to the collapsed sleeve (0.010-0.028 mm
      diametral) and the 0.5-1.5 MPa contact window follow from this tolerance;
    - roundness 2 um peak-to-peak (1 um amplitude): the cup conforms to the round sleeve and springs back, so bore
      roundness is released lobing; with the 0.75 um amplitude of the clamped sleeve the RSS is 1.25 um, inside the
      1.5 um lobing limit (loads/README budget allows 3 um for Fr);
    - cylindricity 3 um over the clamped length;
    - wall thickness constant around the circumference to 10 um peak-to-peak (OD turned from the finished bore):
      expansion is ~6.8 um/MPa at 0.6 mm and varies as dt/t, so 10 um (1.7%) gives under 0.1 um of lobing at
      1.5 MPa, about 5% of the 1.5 um limit. EMAG flags wall variation as changing expansion behaviour; it is also
      the term that turns bore-to-OD concentricity into root-depth error. */
class FlexsplineCupBlank extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", 62, "mm"),
    P.quantity("bore", 58.05, "mm"),
    P.quantity("bodyOutside", 59.25, "mm"),
    P.quantity("bandWidth", 8, "mm"),
    P.quantity("height", 40, "mm"),
    P.quantity("endThickness", 2, "mm"),
    P.flag("boreHoned", true),
    P.quantity("boreToleranceUpper", 0.013, "mm"),
    P.quantity("boreRoundness", 0.002, "mm"),
    P.quantity("boreCylindricity", 0.003, "mm"),
    P.quantity("wallVariation", 0.010, "mm")
  ]);
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.hobbing.flexspline-cup-blank",
      parameters(),
      v -> new FlexsplineCupBlank(v)
    ).numbered(_ -> "FLEXSPLINE-CUP-BLANK").procured(_ -> NotSupplied)
      .requiringInputs(["outside", "bore", "bodyOutside", "bandWidth", "height", "endThickness"]));
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super("FLEXSPLINE-CUP-BLANK", "One-piece flexspline cup blank (assumed)", "steel", false, v);
    if (v.get("material") != null) setMaterial(v.token("material"));
    if (parameterNumber("bore") <= 0 || parameterNumber("bodyOutside") <= parameterNumber("bore")
      || parameterNumber("outside") < parameterNumber("bodyOutside")
      || parameterNumber("bandWidth") <= 0 || parameterNumber("endThickness") <= 0 || parameterNumber("height")
      <= parameterNumber("bandWidth") + parameterNumber("endThickness")
      || parameterNumber("boreToleranceUpper") <= 0 || parameterNumber("boreRoundness") <= 0
      || parameterNumber("boreCylindricity") < parameterNumber("boreRoundness")
      || parameterNumber("wallVariation") <= 0
      || parameterNumber("wallVariation") >= parameterNumber("bodyOutside") / 2 - parameterNumber("bore") / 2) throw "invalid flexspline cup dimensions";
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addFacet(new EngineeringMetadata(v));
    addFacet(machinekit.motion.PrecisionErrors.fromValues(v, []));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var tip = parameterNumber("outside") / 2,
      body = parameterNumber("bodyOutside") / 2, inner = parameterNumber("bore") / 2,
      band = parameterNumber("bandWidth"), height = parameterNumber("height"), end = parameterNumber("endThickness");
    return Part.revolve([
      {r: inner, z: 0.0},
      {r: tip, z: 0.0},
      {r: tip, z: band},
      {r: body, z: band},
      {r: body, z: height},
      {r: 0.0, z: height},
      {r: 0.0, z: height - end},
      {r: inner, z: height - end}
    ]);
  }
}

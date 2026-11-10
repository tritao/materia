package machinekit.motion;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Crossed-roller precision bearing. Reference row is Assumed: its outside, bore and height are not a product's. */
class CrossedRollerBearing extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", 120, "mm"),
    P.quantity("bore", 80, "mm"),
    P.quantity("height", 16, "mm"),
    P.text("precisionClass", "P4"),
    P.quantity("radialRunout", 0.002, "mm"),
    P.quantity("axialRunout", 0.002, "mm"),
    P.quantity("momentStiffness", 1000000, "N m/rad")
  ]);
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.motion.crossed-roller-bearing",
      parameters(),
      v -> new CrossedRollerBearing(v)
    ).numbered(v -> 'CR-BEARING-ASSUMED-${Dimension.format(v.number("bore"))}x${Dimension.format(v.number("outside"))}x${Dimension.format(v.number("height"))}-${v.token("precisionClass")}')
      .procured(_ -> Buy(null, null, null)).requiring(v -> [
        machinekit.component.Procurement.ProcurementCodec.requirement("bore", "mm", null, null, v.number("bore")),
        machinekit.component.Procurement.ProcurementCodec.requirement("outside", "mm", null, null, v.number("outside")),
        machinekit.component.Procurement.ProcurementCodec.requirement("height", "mm", null, null, v.number("height")),
        machinekit.component.Procurement.ProcurementCodec.choice("precisionClass", v.token("precisionClass")),
        machinekit.component.Procurement.ProcurementCodec.requirement("radialRunout", "um", null, v.number("radialRunout") * 1000),
        machinekit.component.Procurement.ProcurementCodec.requirement("axialRunout", "um", null, v.number("axialRunout") * 1000),
        machinekit.component.Procurement.ProcurementCodec.requirement("momentStiffness", "N m/rad", v.number("momentStiffness"))
      ]));
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues
    > return EngineeringMetadata.assumedCatalog(
      "Crossed-roller precision bearing",
      "CROSSED-ROLLER-80-120-16",
      parameters(),
      "machinekit/examples/flexspline-hobber/docs/PLAN.md (assumed reference design)"
    );
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super(
      "CROSSED-ROLLER-" + machinekit.component.Dimension.format(v.number("bore")) + "-" + machinekit.component
      .Dimension.format(v.number("outside")) + "-" + machinekit.component.Dimension.format(v.number("height")),
      "Crossed-roller precision bearing (reference envelope)",
      "bearing steel",
      false,
      v
    );
    if (v.get("material") != null) setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v, [{name: "radialRunout", unit: "mm"}, {name: "axialRunout", unit: "mm"}]));
    if (parameterNumber("momentStiffness") <= 0) throw "bearing stiffness must be positive";
  }
  override public function componentType():Null<ComponentType> return recipeType();
  public var radialRunout(get, never):Float;
  function get_radialRunout():Float return parameterNumber("radialRunout");
  public var axialRunout(get, never):Float;
  function get_axialRunout():Float return parameterNumber("axialRunout");
  public var momentStiffness(get, never):Float;
  function get_momentStiffness():Float return parameterNumber("momentStiffness");

  override public function hasGeometry():Bool return true;
  /** Chamfered rings and shields with eight mounting holes through each ring, as RU-type units (`RingParts.bearing`). Envelope detail is the annulus, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):cadkit.modeling.Part return detail
    == Envelope ? ComponentEnvelopes.annular(values()) : RingParts.bearing(
      parameterNumber("outside"),
      parameterNumber("bore"),
      parameterNumber("height"),
      8
    );
  override public function massGeometry():Null<cadkit.modeling.Part> return ComponentEnvelopes.annular(values());
}

package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Made BT30 hob arbor. MAS BT30 shank dimensions are catalogued; hob seat,
 * face key, lock-nut thread and outboard journal are project design inputs. */
class HobArbor extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", 16, "mm"),
    P.quantity("bore", 0, "mm"),
    P.quantity("height", HobberOutboardLayout.ARBOR_HEIGHT, "mm"),
    P.quantity("hobSeatLength", 40, "mm"),
    P.quantity("hobCentreOffset", 60, "mm"),
    P.quantity("supportDiameter", 15, "mm"),
    P.quantity("supportLength", HobberOutboardLayout.JOURNAL_LENGTH, "mm"),
    P.quantity("radialRunout", 0.002, "mm"),
    P.quantity("axialRunout", 0.002, "mm")
  ]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hobbing.hob-arbor", parameters(), v -> new HobArbor(v))
      .numbered(_ -> "HOB-ARBOR-BT30").procured(_ -> Make("turning and grinding")).requiringInputs(["outside", "bore", "height", "hobSeatLength", "supportDiameter"]);
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues
    > return EngineeringMetadata.assumedCatalog(
      "BT30 hob arbor",
      "HOB-ARBOR-BT30",
      parameters(),
      "machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)"
    );
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super("HOB-ARBOR-BT30", "BT30 hob arbor (reference envelope)", "steel", false, v);
    if (v.get("material") != null) setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(ManufacturingRoute.turned(2, "tight", [
      {name: "BT30 taper, AT3; gauge diameter 31.75, 7/24 (grind)", tolerance: 0.002, roughnessRa: 0.2},
      {name: "hob seat and D15 journal; D20 shoulder, fillet <=0.2; M15x1 at rear (grind)", tolerance: 0.002, roughnessRa: 0.2},
      {name: "hob shoulder perpendicular to taper; M16x1 nut thread beyond seat", tolerance: 0.002, roughnessRa: 0.4},
      {name: "integral face key 5 wide x 3 radial x 2 axial (mill at second setup)", tolerance: 0.02, roughnessRa: 1.6}
    ]));
    addFacet(PrecisionErrors.fromValues(v, [{name: "radialRunout", unit: "mm"}, {name: "axialRunout", unit: "mm"}]));
    if (parameterNumber("supportLength") > 0 && (parameterNumber("bore") >= parameterNumber("supportDiameter")
      || parameterNumber("supportDiameter") > parameterNumber("outside"))) throw "invalid support nose section";
    if (parameterNumber("supportDiameter") <= 0 || parameterNumber("supportLength")
      > parameterNumber("height") - parameterNumber("hobSeatLength")) throw "invalid arbor nose seat";
    if (parameterNumber("hobSeatLength") <= 0
      || parameterNumber("hobSeatLength") >= parameterNumber("height")) throw "hob seat exceeds arbor";
    addConnector("gaugeLine", Mount, Solids.axial(0, 0, 0));
    var centre = parameterNumber("hobCentreOffset")
      == 0 ? parameterNumber("height") / 2 : parameterNumber("hobCentreOffset");
    if (centre < parameterNumber("hobSeatLength") / 2 || centre + parameterNumber("hobSeatLength") / 2
      > parameterNumber("height") - parameterNumber("supportLength")) throw "hob seat exceeds usable arbor span";
    addConnector("hobCentre", Axis, Solids.axial(0, 0, centre));
    if (hobStart() <= Bt30SpindleData.HOLDER_GAP + Bt30SpindleData.HOLDER_LENGTH
      || hobEnd() + 8 > parameterNumber("height") - parameterNumber("supportLength")) throw "BT30 arbor shoulder or retaining nut leaves no usable span";
    addConnector("support", Axis, Solids.axial(0, 0, parameterNumber("height")));
  }
  public function interfaceName():String return "BT30";
  override public function fastensByForm():Bool return true;
  public function hobStart():Float return connector("hobCentre").frame.z - parameterNumber("hobSeatLength") / 2;
  public function hobEnd():Float return hobStart() + parameterNumber("hobSeatLength");
  override public function componentType():Null<ComponentType> return recipeType();


  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):cadkit.modeling.Part {
    var length = parameterNumber("supportLength"), height = parameterNumber("height"), start = hobStart();
    var sections = [
      Bt30SpindleData.taper(0),
      cadkit.modeling.Part.cylinderSpan(19, 0, Bt30SpindleData.HOLDER_GAP + Bt30SpindleData.HOLDER_LENGTH),
      cadkit.modeling.Part.cylinderSpan(Bt30SpindleData.HOLDER_FLANGE / 2, Bt30SpindleData.HOLDER_GAP,
        Bt30SpindleData.HOLDER_GAP + Bt30SpindleData.HOLDER_LENGTH),
      cadkit.modeling.Part.cylinderSpan(12, Bt30SpindleData.HOLDER_GAP + Bt30SpindleData.HOLDER_LENGTH, start),
      cadkit.modeling.Part.cylinderSpan(parameterNumber("outside") / 2, start, height - length),
      // Integral face key: 5 wide, 3 radial engagement, 2 axial engagement,
      // ASSUMED DIN-138-style end slot. The bore remains plain for centring.
      cadkit.modeling.Part.box(3.5, 5, 2).translated(new cadkit.modeling.Vector(9.25, 0, start))
    ];
    if (length > 0) {
      sections.push(cadkit.modeling.Part.cylinderSpan(10, height - length - 5, height - length));
      sections.push(cadkit.modeling.Part.cylinderSpan(parameterNumber("supportDiameter") / 2, height - length, height));
    }
    var tools = [cadkit.modeling.Part.cylinderSpan(6, -Bt30SpindleData.TAPER_LENGTH - 0.1, -Bt30SpindleData.TAPER_LENGTH + 20)];
    // MAS drive-key slots: 16.1 wide, floor 16.3 from axis. Axial
    // engagement depth is an assumed envelope matched to the bought dogs.
    for (side in [-1, 1]) tools.push(cadkit.modeling.Part.box(12, Bt30SpindleData.DRIVE_SLOT_WIDTH,
      Bt30SpindleData.DRIVE_DOG_LENGTH + 0.2).translated(new cadkit.modeling.Vector(side * (Bt30SpindleData.DRIVE_SLOT_FLOOR + 6), 0, -0.1)));
    // M12 pull-stud thread (major bore shown); selected 45-degree stud stays with
    // the arbor. Supplier-specific head geometry must be matched at purchase.
    if (parameterNumber("bore") > 0) tools.push(cadkit.modeling.Part.cylinderSpan(parameterNumber("bore") / 2, -49, height + 1));
    return Solids.cut(Solids.union(sections), tools);
  }
}

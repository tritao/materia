package machinekit.hobbing;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;

/** Stationary envelope of the purchased RobotDigg D80 BT30 cartridge. The
 * purchase includes its rotor, bearings and drawbar. Only diameter, taper type,
 * speed and bearing family are verified; axial envelope and mass are ASSUMED. */
class SpindleHousing extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", Bt30SpindleData.OUTSIDE, "mm"),
    P.quantity("height", Bt30SpindleData.GAUGE_Z, "mm")
  ]);
  public static inline final CLAMP_FROM = Bt30SpindleData.CLAMP_FROM;
  public static inline final CLAMP_TO = Bt30SpindleData.CLAMP_TO;
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hobbing.spindle-housing", parameters(), v -> new SpindleHousing(v))
      .versioned(2).numbered(_ -> Bt30SpindleData.DESIGNATION)
      .procured(_ -> Buy("RobotDigg", "2049 option A", "MAS BT30"));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues> return Bt30SpindleData.catalog();
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    v.setToken("source", Bt30SpindleData.SOURCE + "; verified: D80, BT30, 6000 rpm, HTD5M, 7008+7006. ASSUMED: axial geometry, registers, mass, runout and belt-load allowance; linked NT30 drawing not used.");
    super(Bt30SpindleData.DESIGNATION, "RobotDigg D80 BT30 belt spindle, 6000 rpm (axial envelope assumed)", "steel", false, v);
    if (parameterNumber("outside") != Bt30SpindleData.OUTSIDE || parameterNumber("height") != Bt30SpindleData.GAUGE_Z)
      throw "BT30 spindle envelope differs from the selected purchase configuration";
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, Bt30SpindleData.GAUGE_Z));
    addFacet(new EngineeringMetadata(v));
    addFacet(machinekit.motion.PrecisionErrors.fromValues(v, []));
  }
  override public function carriesJournal():Bool return true;
  override public function componentType():Null<ComponentType> return recipeType();
  public function interfaceName():String return "BT30";
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    // The annulus stands for the complete bought stationary assembly, not a
    // specified race/shaft design. The fixed/rotating partitions touch at the
    // included bearing interface: an air gap would sever the support graph.
    return Solids.cut(Solids.union([
      Part.cylinderSpan(Bt30SpindleData.OUTSIDE / 2, 0, Bt30SpindleData.GAUGE_Z),
      Part.cylinderSpan(Bt30SpindleData.FLANGE_DIAMETER / 2, Bt30SpindleData.FLANGE_FROM, Bt30SpindleData.GAUGE_Z)
    ]), [Part.cylinderSpan(Bt30SpindleData.ROTOR_DIAMETER / 2, -1, Bt30SpindleData.GAUGE_Z + 1)]);
  }
}

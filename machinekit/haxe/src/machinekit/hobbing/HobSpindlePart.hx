package machinekit.hobbing;

import machinekit.component.*;
import cadkit.modeling.Part;

/** Rotating envelope included in the bought BT30 cartridge. No internal bearing
 * design or separately made shaft is implied. The taper is MAS BT30; rotor,
 * rear stub, drawbar/gripper envelope and mass partition are ASSUMED. */
class HobSpindlePart extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([ComponentRecipeSupport.text("role", "rotor")]);
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hobbing.bt30-spindle-rotor", parameters(),
      v -> new HobSpindlePart(v)).numbered(v -> v.token("role") == "rotor" ? "BT30-D80-INCLUDED-ROTOR" : "BT30-M12-45-PULL-STUD-ASSUMED")
      .procured(v -> v.token("role") == "rotor" ? NotSupplied : Buy(null, null, "MAS BT30 45 degree; head geometry assumed")));
  }
  public function new(?overrides:ComponentValues) {
    var v = ComponentRecipeSupport.resolve(parameters(), overrides);
    super(v.token("role") == "rotor" ? "BT30-D80-INCLUDED-ROTOR" : "BT30-M12-45-PULL-STUD-ASSUMED",
      v.token("role") == "rotor" ? "BT30 cartridge rotor (included; assumed envelope)" : "BT30 M12 45-degree pull stud (head envelope assumed)", "steel", false, v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addFacet(new EngineeringMetadata(v));
    addFacet(machinekit.motion.PrecisionErrors.fromValues(v, []));
    if (v.token("role") == "rotor") addFacet(new IncludedInPurchase(Bt30SpindleData.DESIGNATION));
    else if (v.token("role") != "pullStud") throw "unknown BT30 cartridge hardware";
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function fastensByForm():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (values().token("role") == "pullStud") return Solids.union([
      Solids.named(Part.cylinderSpan(6, -60, -33.4), "stud"), Solids.named(Part.cylinderSpan(7, -64, -60), "head")
    ]);
    var g = Bt30SpindleData.GAUGE_Z;
    var body = Solids.union([
      Solids.named(Part.cylinderSpan(Bt30SpindleData.STUB_DIAMETER / 2, Bt30SpindleData.STUB_REAR, 0), "stub"),
      Solids.named(Part.cylinderSpan(Bt30SpindleData.ROTOR_DIAMETER / 2, 0, g), "rotor"),
      // Bought spindle drive dogs: width/radial envelope fit MAS holder slots;
      // axial projection is assumed until the cartridge drawing is received.
      Solids.named(Part.box(6.6, 16.0, Bt30SpindleData.DRIVE_DOG_LENGTH).translated(new cadkit.modeling.Vector(19.65, 0, g)), "drive-dog1"),
      Solids.named(Part.box(6.6, 16.0, Bt30SpindleData.DRIVE_DOG_LENGTH).translated(new cadkit.modeling.Vector(-19.65, 0, g)), "drive-dog2")
    ]);
    // The bought drawbar is inside this envelope. A rear axial access bore is
    // reserved; its actuation/retention forces remain supplier assumptions.
    return Solids.cut(body, [Solids.named(Bt30SpindleData.taper(g), "taper"), Solids.named(Part.cylinderSpan(8, Bt30SpindleData.STUB_REAR - 1, g - Bt30SpindleData.TAPER_LENGTH), "drawbar-bore")]);
  }
}

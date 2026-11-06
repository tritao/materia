package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** Generic inline vacuum sensor with a pressure feedback signal. */
class VacuumPressureSensor extends MachineComponent {

  public function new(tubeOdMm:Float = 4) {
    if (!Math.isFinite(tubeOdMm) || tubeOdMm <= 0) throw "Vacuum pressure sensor needs a positive tube diameter";
    super('VACUUM-PRESSURE-SENSOR-${tubeOdMm}', "Generic inline vacuum pressure sensor", "plastic", true);
    if (Std.isExactType(this, VacuumPressureSensor)) addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addPort({name: "vacuumIn", kind: Vacuum, role: Consumer, iface: PushIn(tubeOdMm), required: true});
    addPort({name: "vacuumOut", kind: Vacuum, role: Supply, iface: PushIn(tubeOdMm), required: false});
    addBridge("vacuumIn", "vacuumOut");
    addPort({name: "pressureSignal", kind: Signal, role: Supply, iface: Plug("analog-vacuum-kpa", 3), required: false});
    addFacet(new VacuumPressureSensorFacet("vacuumIn", "pressureSignal"));
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.pneumatic.vacuum-pressure-sensor",
      [ComponentRecipeSupport.length("tubeOdMm", 4)],
      v -> new VacuumPressureSensor(v.number("tubeOdMm")),
      true
    ).procured(_ -> Buy(null, null, null)).requiring(v -> [
      machinekit.component.Procurement.ProcurementCodec.requirement("tubeOutsideDiameter", "mm", null, null, v.number("tubeOdMm")),
      machinekit.component.Procurement.ProcurementCodec.choice("output", "analog-vacuum-kpa")
    ]));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    VacuumPressureSensor) ? recipeType() : null;

  override public function values():machinekit.component.ComponentValues return new machinekit
    .component.ComponentValues().setNumber("tubeOdMm", switch port("vacuumIn").iface {
    case PushIn(d):
      d;
    case _:
      0;
  }
  ).setToken("material", materialSpec());

  override public function hasGeometry():Bool return true;

  /** Body with its display window, on its port boss (`PneumaticBodies.sensor`). The envelope box is also the mass model. */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail == Envelope ? Part.box(
    20,
    15,
    25
  ) : PneumaticBodies.sensor(
    20,
    15,
    25
  );

  override public function massGeometry():Null<Part> return Part.box(20, 15, 25);
}

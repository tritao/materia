package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.fluid.ValveSpec;
import machinekit.fluid.ValveControlSpec;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed 4/3 hydraulic valve block: agreeing coils close every work port.
 * P is the regulated feed, T the tank return, and A/B the cylinder work outlets.
 */
class HydraulicValve extends MachineComponent implements ValveSpec {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public function new() {
    super("HYDRAULIC-VALVE-CLOSED-CENTRE", "Assumed closed-centre hydraulic valve block", "steel", true);
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addPort({name: "P", kind: Hydraulic, role: Consumer, iface: Thread("G1/2"), required: true});
    addPort({name: "T", kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
    for (name in ["A", "B"]) {
      addPort({name: name, kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
      addBridge("P", name);
    }
    for (name in ["coilA", "coilB"]) addPort({
      name: name,
      kind: Signal,
      role: Consumer,
      iface: Plug("24V-coil", 2),
      required: false
    });
  }
  public function controlSpec():ValveControlSpec return ClosedCentre;
  override public function valveSpec():Null<ValveSpec> return this;
  override public function hasGeometry():Bool return true;
  /** A double-solenoid spool valve on its block. Envelope detail is the block, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail
    == Envelope ? massGeometry() : machinekit.pneumatic.PneumaticBodies.valve(
      120,
      60,
      60,
      70,
      2,
      [-22, 0, 22],
      7
    );

  override public function massGeometry():Null<Part> return Solids.named(Part.box(120,
    60, 60), "hydraulic-valve-block");
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.pressbrake.hydraulic-valve",
      [],
      _ -> new HydraulicValve()
    ).numbered(_ -> "HYD-VALVE-ASSUMED-4/3-CLOSED").procured(_ -> Buy(null, null, null)).requiring(_ -> [
      machinekit.component.Procurement.ProcurementCodec.choice("configuration", "4/3 closed centre"),
      machinekit.component.Procurement.ProcurementCodec.choice("portThread", "G1/2"),
      machinekit.component.Procurement.ProcurementCodec.requirement("coilVoltage", "V", null, null, 24)
    ]));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
}

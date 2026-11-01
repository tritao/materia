package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed controller cabinet; program slots carry no invented tightening recipes. */
class TighteningController extends MachineComponent {
  public function new(?programs:Array<Int>, controlInterface:TighteningControlInterface = DiscreteIO) {
    super("CUSTOM-TIGHTENING-CONTROLLER", "Tightening controller cabinet", "steel", true);
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), [
      machinekit.component.Procurement.ProcurementCodec.choice("controlInterface", Std.string(controlInterface)),
      machinekit.component.Procurement.ProcurementCodec.requirement("inputVoltage", "V", null, null, 230),
      machinekit.component.Procurement.ProcurementCodec.requirement("programs", "1", programs == null ? 4 : programs.length)
    ]));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addPort({
      name: "mains",
      kind: ElectricalPower,
      role: Consumer,
      iface: ScrewdrivingInterfaces.mains(),
      required: true
    });
    addPort({
      name: "toolPower",
      kind: ElectricalPower,
      role: Supply,
      iface: ScrewdrivingInterfaces.tool(),
      required: false
    });
    addPort({name: "tool", kind: Signal, role: Supply, iface: ScrewdrivingInterfaces.tool(), required: false});
    addPort({
      name: "control",
      kind: Signal,
      role: Consumer,
      iface: ScrewdrivingInterfaces.control(controlInterface),
      required: true
    });
    addFacet(new TighteningSupplyFacet(programs == null ? [1, 2, 3, 4] : programs, controlInterface));
  }
  override public function revisionFingerprint():Null<String> {
    var supply = TighteningSupplyFacet.of(this);
    if (supply == null) return null;
    return machinekit.component.BomRevision.fingerprint(["TighteningController", materialSpec(),
      Std.string(supply.controlInterface), supply.programs.join(",")].join("|"), 1);
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(250, 200, 400);
}

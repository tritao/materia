package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Rotating tool nose. Its tip connector includes the fitted bit's reach. */
class ScrewdriverCarrier extends MachineComponent {
  public final bit:ScrewdriverBit;

  public function new(bit:ScrewdriverBit, maxTorque:Float, maxSpeed:Float, strokeMm:Float, springRate:Float) {
    if (bit == null) throw "Screwdriver carrier needs its fitted bit";
    super("CUSTOM-SCREWDRIVER-CARRIER", "Rotating screwdriver nose", "steel", true);
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), [
      machinekit.component.Procurement.ProcurementCodec.requirement("maxTorque", "N m", maxTorque),
      machinekit.component.Procurement.ProcurementCodec.requirement("maxSpeed", "rad/s", maxSpeed),
      machinekit.component.Procurement.ProcurementCodec.requirement("stroke", "mm", null, null, strokeMm)
    ]));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.bit = bit;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("bit", Mount, Solids.axial(0, 0, 15));
    addConnector("tcp", Mount, materia.assembly.AssemblyFrames.translation(0, 0, 15 + bit.length));
    addPort({name: "control", kind: Signal, role: Consumer, iface: ScrewdrivingInterfaces.tool(), required: true});
    addFacet(new ScrewdriverFacet(
      "tcp",
      "control",
      maxTorque,
      maxSpeed,
      strokeMm,
      springRate,
      bit,
      "feed",
      "spindle",
      "stroke"
    ));
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Part.cylinderSpan(8, 0, 15);
}

package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed motor/gearbox or slide collar, kept as separate bodies for the real tool joints. */
class ScrewdriverHousing extends MachineComponent {
  public final length:Float;
  public final radius:Float;
  final powered:Bool;

  public function new(name:String, length:Float, radius:Float, powered:Bool = false, ?maxTorque:Float, ?maxSpeed:Float,
    ?strokeMm:Float, ?springRate:Float, ?purchaseNumber:String) {
    if (!Math.isFinite(length) || length <= 0
      || !Math.isFinite(radius) || radius <= 0) throw "Screwdriver body needs finite positive dimensions";
    var number = purchaseNumber == null ? customDesignation(powered ? "SCREWDRIVER-DRIVE" : "SCREWDRIVER-SLIDE",
      {length:length, radius:radius, maxTorque:maxTorque, maxSpeed:maxSpeed, stroke:strokeMm, springRate:springRate}) : purchaseNumber;
    super(number, name, "aluminium 6061", true);
    var terms = [machinekit.component.Procurement.ProcurementCodec.choice("kind", powered ? "powered screwdriver" : "feed slide"),
      machinekit.component.Procurement.ProcurementCodec.requirement("length", "mm", null, null, length),
      machinekit.component.Procurement.ProcurementCodec.requirement("radius", "mm", null, null, radius)];
    if (maxTorque != null) terms.push(machinekit.component.Procurement.ProcurementCodec.requirement("maxTorque", "N m", maxTorque));
    if (maxSpeed != null) terms.push(machinekit.component.Procurement.ProcurementCodec.requirement("maxSpeed", "rad/s", maxSpeed));
    if (strokeMm != null) terms.push(machinekit.component.Procurement.ProcurementCodec.requirement("stroke", "mm", null, null, strokeMm));
    if (springRate != null) terms.push(machinekit.component.Procurement.ProcurementCodec.requirement("springRate", "N/mm", springRate));
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), terms));
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    this.length = length;
    this.radius = radius;
    this.powered = powered;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("end", Mount, Solids.axial(0, 0, length));
    if (powered) addPort({
      name: "power",
      kind: ElectricalPower,
      role: Consumer,
      iface: ScrewdrivingInterfaces.tool(),
      required: true
    });
    addPort({name: "control", kind: Signal, role: Consumer, iface: ScrewdrivingInterfaces.tool(), required: true});
    addPort({name: "bitControl", kind: Signal, role: Supply, iface: ScrewdrivingInterfaces.tool(), required: false});
    addPort({name: "vacuum", kind: Vacuum, role: Consumer, iface: ScrewdrivingInterfaces.vacuum(), required: true});
    addPort({name: "bitVacuum", kind: Vacuum, role: Supply, iface: ScrewdrivingInterfaces.vacuum(), required: false});
    addBridge("control", "bitControl");
    addBridge("vacuum", "bitVacuum");
  }
  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    ["screwdriver-housing", materialSpec(), machinekit.component.ComponentNumberKey.encode(length),
      machinekit.component.ComponentNumberKey.encode(radius), powered ? "powered" : "slide"].join("|"), 1);
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Part.cylinderSpan(radius, 0, length);
}

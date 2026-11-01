package machinekit.power;

import cadkit.modeling.Vector;
import machinekit.motion.PowerSupply;

/** An isolated DC supply whose input is explicitly wired to the floating pack. */
class IsolatedDcConverter extends PowerSupply {
  public final inputVolts:Float;
  public final efficiency:Float;
  public final massKg:Float;
  public function new(
    inputVolts:Float,
    outputVolts:Float,
    outputAmps:Float,
    outlets:Int,
    width:Float = 120,
    height:Float = 60,
    depth:Float = 30,
    efficiency:Float = 0.95,
    massKg:Float = 2,
    regenerativePower:Float = 0
  ) {
    if (!Math.isFinite(inputVolts) || inputVolts <= 0 || !Math.isFinite(efficiency) || efficiency <= 0
      || efficiency > 1 || !Math.isFinite(massKg) || massKg <= 0) throw "Invalid isolated DC converter ratings";
    super(new DcVoltage(outputVolts), new DcCurrent(outputAmps), outlets, width, height, depth, regenerativePower);
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), [
      machinekit.component.Procurement.ProcurementCodec.requirement("inputVoltage", "V", null, null, inputVolts),
      machinekit.component.Procurement.ProcurementCodec.requirement("outputVoltage", "V", null, null, outputVolts),
      machinekit.component.Procurement.ProcurementCodec.requirement("outputCurrent", "A", outputAmps),
      machinekit.component.Procurement.ProcurementCodec.requirement("outlets", "1", outlets),
      machinekit.component.Procurement.ProcurementCodec.requirement("regenerativePower", "W", regenerativePower),
      machinekit.component.Procurement.ProcurementCodec.requirement("efficiency", "1", efficiency),
      machinekit.component.Procurement.ProcurementCodec.requirement("width", "mm", null, width),
      machinekit.component.Procurement.ProcurementCodec.requirement("height", "mm", null, height),
      machinekit.component.Procurement.ProcurementCodec.requirement("depth", "mm", null, depth)
    ]));
    this.inputVolts = inputVolts;
    this.efficiency = efficiency;
    this.massKg = massKg;
    addPort({name: "dc", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: true});
    for (index in 1...outlets + 1) addConversion("dc", 'power$index');
    declareMass(massKg, new Vector(0, 0, depth / 2));
  }
  override function get_bom():machinekit.component.BomItem {
    var item = super.get_bom();
    var f = machinekit.component.Dimension.format;
    item.partNumber = 'DCDC-${f(inputVolts)}-${f(voltage.volts())}V${f(current.amps())}A-${outlets}O-'
      + '${f(width)}x${f(height)}x${f(depth)}-R${f(regenerativePower)}-E${f(efficiency * 100)}';
    return item;
  }
  override public function revisionFingerprint():Null<String> {
    var key = machinekit.component.ComponentNumberKey.encode;
    return machinekit.component.BomRevision.fingerprint(["isolated-dc-converter", materialSpec(), key(inputVolts),
      key(voltage.volts()), key(current.amps()), Std.string(outlets), key(width), key(height), key(depth),
      key(regenerativePower), key(efficiency), key(massKg)].join("|"), 1);
  }
  public function inputPower(watts:Float):Float {
    if (!Math.isFinite(watts)
      || watts < 0 || watts > voltage.volts() * current.amps()) throw "DC converter load exceeds its output rating";
    return watts / efficiency;
  }
}

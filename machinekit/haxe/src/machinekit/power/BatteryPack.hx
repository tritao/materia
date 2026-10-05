package machinekit.power;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.ElectricalSource;

/** Insulated pack enclosure; electrical terminals are isolated from its mounting chassis. */
class BatteryPack extends MachineComponent implements ElectricalSource {
  public final nominalVolts:Float;
  public final capacityWh:Float;
  public final length:Float;
  public final width:Float;
  public final height:Float;
  public final massKg:Float;
  public final outlets:Int;
  public function new(nominalVolts:Float, capacityWh:Float, length:Float, width:Float,
      height:Float, massKg:Float, outlets:Int = 1) {
    for (value in [nominalVolts, capacityWh, length, width, height, massKg])
      if (!Math.isFinite(value) || value <= 0) throw "Battery needs finite positive ratings, dimensions and mass";
    if (outlets < 1 || outlets > 64) throw "Battery needs 1 to 64 supply ports";
    super('BATTERY-${Dimension.format(nominalVolts)}V-${Dimension.format(capacityWh)}WH-' +
      '${Dimension.format(length)}x${Dimension.format(width)}x${Dimension.format(height)}',
      "Insulated battery pack", "plastic", true);
    this.nominalVolts = nominalVolts;
    this.capacityWh = capacityWh;
    this.length = length;
    this.width = width;
    this.height = height;
    this.massKg = massKg;
    this.outlets = outlets;
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    for (index in 1...outlets + 1)
      addPort({name: 'power$index', kind: ElectricalPower, role: Supply, iface: Unspecified, required: false});
    addFacet(new BatteryStorage(nominalVolts, capacityWh));
    declareMass(massKg, new Vector(0, 0, height / 2));
  }
  public function outputVoltage(name:String):Float {
    var output = port(name);
    if (output.kind != ElectricalPower || output.role != Supply) throw "Not a battery output";
    return nominalVolts;
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(length, width, height);
}

package machinekit.power;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Pure-sine, single-phase inverter. It converts power; it is not a DC source or a wire bridge. */
class Inverter extends MachineComponent {
  public final inputVolts:Float;
  public final outputVolts:Float;
  public final continuousWatts:Float;
  public final efficiency:Float;
  public final length:Float;
  public final width:Float;
  public final height:Float;
  public final massKg:Float;
  public function new(inputVolts:Float, outputVolts:Float, continuousWatts:Float,
      length:Float, width:Float, height:Float, massKg:Float, efficiency:Float = 0.92) {
    for (value in [inputVolts, outputVolts, continuousWatts, length, width, height, massKg])
      if (!Math.isFinite(value) || value <= 0) throw "Inverter needs finite positive ratings, dimensions and mass";
    if (!Math.isFinite(efficiency) || efficiency <= 0 || efficiency > 1) throw "Invalid inverter efficiency";
    if (outputVolts != 230) throw "This inverter profile supplies 230 V single-phase mains";
    super('INVERTER-${Dimension.format(inputVolts)}V-${Dimension.format(outputVolts)}V-${Dimension.format(continuousWatts)}W',
      "Pure-sine single-phase inverter", "aluminium 6061", true);
    this.inputVolts = inputVolts;
    this.outputVolts = outputVolts;
    this.continuousWatts = continuousWatts;
    this.efficiency = efficiency;
    this.length = length;
    this.width = width;
    this.height = height;
    this.massKg = massKg;
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addPort({name: "dc", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: true});
    addPort({name: "mains", kind: ElectricalPower, role: Supply, iface: Plug("mains-230v-1ph", 3), required: false});
    addConversion("dc", "mains");
    declareMass(massKg, new Vector(0, 0, height / 2));
  }
  public function inputPower(outputWatts:Float):Float {
    if (!Math.isFinite(outputWatts) || outputWatts < 0 || outputWatts > continuousWatts)
      throw "Inverter demand exceeds its continuous rating";
    return outputWatts / efficiency;
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(length, width, height);
}

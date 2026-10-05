package machinekit.power;

import cadkit.modeling.Vector;
import machinekit.motion.PowerSupply;

/** An isolated DC supply whose input is explicitly wired to the floating pack. */
class IsolatedDcConverter extends PowerSupply {
  public final inputVolts:Float;
  public final efficiency:Float;
  public function new(inputVolts:Float, outputVolts:Float, outputAmps:Float, outlets:Int,
      width:Float = 120, height:Float = 60, depth:Float = 30, efficiency:Float = 0.95, massKg:Float = 2) {
    if (!Math.isFinite(inputVolts) || inputVolts <= 0 || !Math.isFinite(efficiency) ||
        efficiency <= 0 || efficiency > 1 || !Math.isFinite(massKg) || massKg <= 0)
      throw "Invalid isolated DC converter ratings";
    super(outputVolts, outputAmps, outlets, width, height, depth);
    this.inputVolts = inputVolts;
    this.efficiency = efficiency;
    addPort({name: "dc", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: true});
    for (index in 1...outlets + 1) addConversion("dc", 'power$index');
    declareMass(massKg, new Vector(0, 0, depth / 2));
  }
  public function inputPower(watts:Float):Float {
    if (!Math.isFinite(watts) || watts < 0 || watts > voltage * current)
      throw "DC converter load exceeds its output rating";
    return watts / efficiency;
  }
}

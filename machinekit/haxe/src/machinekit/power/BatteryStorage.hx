package machinekit.power;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Stored electrical energy; ratings belong to the CAD, state of charge to the runtime. */
class BatteryStorage implements ComponentFacet {
  public final nominalVolts:Float;
  public final capacityWh:Float;
  public function new(nominalVolts:Float, capacityWh:Float) {
    for (value in [nominalVolts, capacityWh])
      if (!Math.isFinite(value) || value <= 0) throw "Battery storage ratings must be finite and positive";
    this.nominalVolts = nominalVolts;
    this.capacityWh = capacityWh;
  }
  public function check(component:MachineComponent):Void {
    var supply = false;
    for (port in component.ports()) if (port.kind == ElectricalPower && port.role == Supply) supply = true;
    if (!supply) throw "Battery storage needs an electrical supply port";
  }
  public function describe():String return 'battery storage $nominalVolts V, $capacityWh Wh';
  public static function of(component:MachineComponent):Null<BatteryStorage> {
    for (facet in component.facets()) if (Std.isOfType(facet, BatteryStorage)) return cast facet;
    return null;
  }
}

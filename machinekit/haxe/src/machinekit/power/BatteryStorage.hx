package machinekit.power;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Stored electrical energy; ratings belong to the CAD, state of charge to the runtime. */
class BatteryStorage implements ComponentFacet {
  public final nominalVolts:Float;
  public final capacityWh:Float;
  /** Open-circuit voltage at full and empty state of charge. */
  public final fullVoltage:Float;
  public final emptyVoltage:Float;
  /** Minimum terminal voltage at which the pack can supply its loads. */
  public final cutoffVoltage:Float;
  /** Pack-level resistance used to compute voltage sag under load. */
  public final internalResistanceOhms:Float;
  public final maxDischargeCurrentA:Float;
  public final maxChargePowerW:Float;
  public final chargeEfficiency:Float;

  public function new(nominalVolts:Float, capacityWh:Float, ?fullVoltage:Float,
      ?emptyVoltage:Float, ?cutoffVoltage:Float, internalResistanceOhms:Float = 0.02,
      maxDischargeCurrentA:Float = 250.0, maxChargePowerW:Float = 5000.0,
      chargeEfficiency:Float = 0.95) {
    var full = fullVoltage == null ? nominalVolts * 1.1375 : fullVoltage;
    var empty = emptyVoltage == null ? nominalVolts * 0.8333333333333334 : emptyVoltage;
    var cutoff = cutoffVoltage == null ? nominalVolts * 0.875 : cutoffVoltage;
    for (value in [nominalVolts, capacityWh, full, empty, cutoff,
        internalResistanceOhms, maxDischargeCurrentA, maxChargePowerW])
      if (!Math.isFinite(value) || value <= 0) throw "Battery storage ratings must be finite and positive";
    if (full <= nominalVolts || nominalVolts <= empty || cutoff <= empty ||
        cutoff >= full || !Math.isFinite(chargeEfficiency) ||
        chargeEfficiency <= 0 || chargeEfficiency > 1)
      throw "Battery voltage curve, cutoff and charge efficiency are invalid";
    this.nominalVolts = nominalVolts;
    this.capacityWh = capacityWh;
    this.fullVoltage = full;
    this.emptyVoltage = empty;
    this.cutoffVoltage = cutoff;
    this.internalResistanceOhms = internalResistanceOhms;
    this.maxDischargeCurrentA = maxDischargeCurrentA;
    this.maxChargePowerW = maxChargePowerW;
    this.chargeEfficiency = chargeEfficiency;
  }
  public function check(component:MachineComponent):Void {
    var supply = false;
    for (port in component.ports()) if (port.kind == ElectricalPower && port.role == Supply) supply = true;
    if (!supply) throw "Battery storage needs an electrical supply port";
  }
  public function describe():String return 'battery storage $nominalVolts V, $capacityWh Wh, ' +
    '$emptyVoltage-$fullVoltage V, $cutoffVoltage V cutoff';
  public static function of(component:MachineComponent):Null<BatteryStorage> {
    for (facet in component.facets()) if (Std.isOfType(facet, BatteryStorage)) return cast facet;
    return null;
  }
}

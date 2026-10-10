package machinekit.power;

import haxe.Int64;

/**
 * Deterministic runtime model of a battery pack. CAD supplies the ratings;
 * callers supply measured bus demand and charging power for each simulation tick.
 */
class BatterySimulator {
  public final batteryId:String;
  public final storage:BatteryStorage;
  public final initialChargeFraction:Float;
  public var supplyAvailable(default, null):Bool;
  public var chargeFraction(default, null):Float;

  var state:BatteryReading;

  public function new(batteryId:String, storage:BatteryStorage, ?initialChargeFraction:Float = 1.0) {
    if (batteryId == null || batteryId.length == 0 || storage == null ||
        !Math.isFinite(initialChargeFraction) || initialChargeFraction < 0 || initialChargeFraction > 1)
      throw "Battery simulation needs a pack and valid initial charge";
    this.batteryId = batteryId;
    this.storage = storage;
    this.initialChargeFraction = initialChargeFraction;
    chargeFraction = initialChargeFraction;
    var voltage = openCircuitVoltage();
    supplyAvailable = voltage >= storage.cutoffVoltage && chargeFraction > 0;
    state = observation(voltage, 0, Int64.ofInt(0));
  }

  /**
   * Advance one tick. `loadWatts` is the measured demand at the pack bus;
   * `chargerWatts` is available output at the charger connector before pack
   * charge losses. Positive current in the resulting state means discharge.
   */
  public function advance(loadWatts:Float, chargerWatts:Float, durationSeconds:Float,
      timestampNs:Int64):BatteryReading {
    if (!Math.isFinite(loadWatts) || loadWatts < 0 || !Math.isFinite(chargerWatts) || chargerWatts < 0 ||
        !Math.isFinite(durationSeconds) || durationSeconds < 0 || timestampNs == null ||
        Int64.compare(timestampNs, state.sourceTimestampNs) < 0)
      throw "Battery simulation received an invalid load, duration or timestamp";
    if (chargerWatts > storage.maxChargePowerW)
      throw "Battery charger exceeds the pack's charge-power rating";

    var openCircuit = openCircuitVoltage();
    var power = loadWatts - chargerWatts * storage.chargeEfficiency;
    var current = currentForPower(openCircuit, power, storage);
    var voltage = openCircuit - current * storage.internalResistanceOhms;
    var currentAllowed = current >= 0
      ? current <= storage.maxDischargeCurrentA
      : -current <= storage.maxChargePowerW / storage.emptyVoltage;
    supplyAvailable = currentAllowed && voltage >= storage.cutoffVoltage &&
      (chargeFraction > 0 || current < 0);

    // A tripped pack exposes its sagged voltage, but stops integrating a load
    // it can no longer support. Charging remains possible below the cutoff.
    if ((current >= 0 && supplyAvailable) || current < 0) {
      var storedWattHours = -openCircuit * current * durationSeconds / 3600.0;
      chargeFraction = Math.max(0.0, Math.min(1.0,
        chargeFraction + storedWattHours / storage.capacityWh));
    }
    if (chargeFraction >= 1.0 && current < 0) {
      current = 0.0;
    }
    voltage = openCircuitVoltage() - current * storage.internalResistanceOhms;
    supplyAvailable = supplyAvailable && voltage >= storage.cutoffVoltage &&
      (chargeFraction > 0 || current < 0);
    state = observation(voltage, current, timestampNs);
    return state;
  }

  public function reading():BatteryReading return state;

  public function reset():Void {
    chargeFraction = initialChargeFraction;
    var voltage = openCircuitVoltage();
    supplyAvailable = voltage >= storage.cutoffVoltage && chargeFraction > 0;
    state = observation(voltage, 0, Int64.ofInt(0));
  }

  static function currentForPower(openCircuitVolts:Float, powerWatts:Float,
      storage:BatteryStorage):Float {
    if (powerWatts == 0.0) return 0.0;
    // The denominator form avoids cancellation for small positive loads.
    var resistance = storage.internalResistanceOhms;
    var discriminant = openCircuitVolts * openCircuitVolts - 4.0 * resistance * powerWatts;
    if (discriminant < 0.0) return powerWatts / openCircuitVolts;
    return 2.0 * powerWatts / (openCircuitVolts + Math.sqrt(discriminant));
  }

  function openCircuitVoltage():Float
    return storage.emptyVoltage + (storage.fullVoltage - storage.emptyVoltage) * chargeFraction;

  function observation(voltage:Float, current:Float, timestampNs:Int64):BatteryReading
    return new BatteryReading(batteryId, chargeFraction, Math.max(0.0, voltage), current, 25.0,
      storage.capacityWh * chargeFraction, timestampNs);
}

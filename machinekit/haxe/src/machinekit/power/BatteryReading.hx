package machinekit.power;

import haxe.Int64;

/** Local simulation observation of a battery pack; transport clocks are added by its runtime adapter. */
class BatteryReading {
  public final batteryId:String;
  public final chargeFraction:Float;
  public final voltageVolts:Float;
  /** Positive current means discharge; negative current means charging. */
  public final currentAmps:Float;
  public final temperatureCelsius:Float;
  public final remainingEnergyWattHours:Float;
  public final sourceTimestampNs:Int64;

  public function new(batteryId:String, chargeFraction:Float, voltageVolts:Float,
      currentAmps:Float, temperatureCelsius:Float, remainingEnergyWattHours:Float,
      sourceTimestampNs:Int64) {
    this.batteryId = batteryId;
    this.chargeFraction = chargeFraction;
    this.voltageVolts = voltageVolts;
    this.currentAmps = currentAmps;
    this.temperatureCelsius = temperatureCelsius;
    this.remainingEnergyWattHours = remainingEnergyWattHours;
    this.sourceTimestampNs = sourceTimestampNs;
  }
}

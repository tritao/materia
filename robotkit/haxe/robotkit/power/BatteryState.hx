package robotkit.power;

import haxe.Int64;

/** Immutable battery observation with charge fraction and clock provenance. */
class BatteryState {
  public final batteryId:String;
  public final chargeFraction:Float;
  public final voltageVolts:Float;
  /** Positive current means discharge; negative current means charging. */
  public final currentAmps:Float;
  public final temperatureCelsius:Float;
  public final remainingEnergyWattHours:Null<Float>;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;

  public function new(batteryId:String, chargeFraction:Float, voltageVolts:Float,
      currentAmps:Float, temperatureCelsius:Float,
      sourceTimestampNs:Int64, receivedTimestampNs:Int64,
      sourceClockId:String, receivedClockId:String,
      ?remainingEnergyWattHours:Float) {
    if (batteryId == null || batteryId.length == 0 || !Math.isFinite(chargeFraction) ||
        chargeFraction < 0.0 || chargeFraction > 1.0 || !Math.isFinite(voltageVolts) ||
        voltageVolts < 0.0 || !Math.isFinite(currentAmps) ||
        !Math.isFinite(temperatureCelsius) || sourceClockId == null || sourceClockId.length == 0 ||
        receivedClockId == null || receivedClockId.length == 0 ||
        (remainingEnergyWattHours != null &&
          (!Math.isFinite(remainingEnergyWattHours) || remainingEnergyWattHours < 0.0)))
      throw "Battery state values are invalid";
    this.batteryId = batteryId;
    this.chargeFraction = chargeFraction;
    this.voltageVolts = voltageVolts;
    this.currentAmps = currentAmps;
    this.temperatureCelsius = temperatureCelsius;
    this.remainingEnergyWattHours = remainingEnergyWattHours;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
  }
}

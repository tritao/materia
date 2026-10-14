package tests;

import haxe.Int64;
import robotkit.core.ForceTorqueMeasurement;
import robotkit.core.SensorFrame;
import robotkit.time.ClockMapping;
import robotkit.time.ClockMappings;

class ForceTorqueMeasurementTests {
  static var assertions = 0;
  static function check(value:Bool, reason:String):Void {
    assertions++;
    if (!value) throw reason;
  }
  static function frame(?sensor:String = "ft/tool", ?frameId:String = "tool/ft", ?kind:String = "force_torque",
    ?clock:String = "sensor.clock", ?sequence:Int = 1, ?timestamp:Int = 100000,
    ?values:Array<Float>):SensorFrame {
    return new SensorFrame(sensor, kind, frameId, Int64.ofInt(sequence), Int64.ofInt(timestamp),
      values == null ? [1, 2, 3, 4, 5, 6] : values, null, null, null, null, clock);
  }
  static function read(sample:SensorFrame, now:Int, maximumAge:Int, ?mappings:ClockMappings):ForceTorqueMeasurement
    return ForceTorqueMeasurement.fromFrame(sample, "ft/tool", "tool/ft", Int64.ofInt(now),
      "robot.clock", Int64.ofInt(maximumAge), mappings);
  static function rejected(action:Void->ForceTorqueMeasurement):Bool {
    try { var value = action(); } catch (_:Dynamic) return true;
    return false;
  }
  public static function run():Int {
    assertions = 0;
    var mappings = new ClockMappings();
    mappings.add(new ClockMapping("sensor.clock", "robot.clock", Int64.ofInt(1000), 0,
      Int64.ofInt(500), Int64.ofInt(0), "commissioned force/torque clock map"));
    var measured = read(frame(), 102000, 5000, mappings);
    check(measured.forceUnit == "N" && measured.torqueUnit == "N*m" && measured.wrench.force.x == 1 &&
      measured.wrench.force.z == 3 && measured.wrench.torque.x == 4 && measured.wrench.torque.z == 6,
      "Six force/torque channels are retained with explicit SI units");
    check(measured.robotTimestampNs == Int64.ofInt(101000) && measured.clockErrorNs == Int64.ofInt(500) &&
      measured.sourceClockId == "sensor.clock" && measured.frameId == "tool/ft",
      "Source identity, frame and mapped acquisition interval remain attached to the wrench");
    check(rejected(() -> read(frame(), 102000, 500, mappings)),
      "A force sample outside the configured age bound is rejected");
    check(rejected(() -> read(frame(), 101250, 5000, mappings)),
      "A mapped sample whose uncertainty interval extends into the future is rejected");
    check(rejected(() -> read(frame("other", "tool/ft"), 102000, 5000, mappings)),
      "A wrench from a different physical sensor cannot be substituted");
    check(rejected(() -> read(frame("ft/tool", "base"), 102000, 5000, mappings)),
      "A wrench in a different coordinate frame cannot be substituted");
    check(rejected(() -> read(frame("ft/tool", "tool/ft", "imu"), 102000, 5000, mappings)),
      "Non-force/torque sensor kinds cannot authorize compliance motion");
    check(rejected(() -> read(frame("ft/tool", "tool/ft", "force_torque", "sensor.clock", 0), 102000, 5000, mappings)),
      "A source without a positive acquisition sequence is rejected");
    check(rejected(() -> read(frame("ft/tool", "tool/ft", "force_torque", "sensor.clock", 2, 100000,
      [1, 2, Math.NaN, 4, 5, 6]), 102000, 5000, mappings)),
      "Non-finite wrench values are rejected");
    check(rejected(() -> read(frame(), 100000, 5000)),
      "An unmapped independent sensor clock cannot supply a wrench");
    return assertions;
  }
}

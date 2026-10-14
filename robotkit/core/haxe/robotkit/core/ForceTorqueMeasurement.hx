package robotkit.core;

import haxe.Int64;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;
import robotkit.time.ClockMapping;
import robotkit.time.ClockMappings;
import robotkit.time.MappedTimestamp;

/**
 * One fresh SI force/torque observation. Values are the external wrench acting
 * on the sensor, in N and N*m, expressed at the named sensor frame; source and
 * mapped robot-clock timestamps stay with the wrench.
 */
class ForceTorqueMeasurement {
  public static inline final KIND = "force_torque";
  public final sensorId:String;
  public final frameId:String;
  public final sequence:Int64;
  public final sourceTimestampNs:Int64;
  public final sourceClockId:String;
  public final robotTimestampNs:Int64;
  public final clockErrorNs:Int64;
  public final forceUnit:String;
  public final torqueUnit:String;
  public final wrench:Wrench3;

  function new(sensorId:String, frameId:String, sequence:Int64, sourceTimestampNs:Int64,
    sourceClockId:String, robotTimestampNs:Int64, clockErrorNs:Int64, wrench:Wrench3) {
    this.sensorId = sensorId;
    this.frameId = frameId;
    this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.sourceClockId = sourceClockId;
    this.robotTimestampNs = robotTimestampNs;
    this.clockErrorNs = clockErrorNs;
    forceUnit = "N";
    torqueUnit = "N*m";
    this.wrench = wrench;
  }

  /** Parse one mounted sensor sample and prove its entire clock interval is fresh. */
  public static function fromFrame(frame:SensorFrame, expectedSensorId:String, expectedFrameId:String,
    nowNs:Int64, robotClockId:String, maximumAgeNs:Int64,
    ?mappings:ClockMappings):ForceTorqueMeasurement {
    if (frame == null || expectedSensorId == null || expectedSensorId.length == 0 ||
      expectedFrameId == null || expectedFrameId.length == 0 || frame.sensorId != expectedSensorId ||
      frame.frameId != expectedFrameId || frame.kind != KIND || frame.image != null ||
      frame.values.length != 6 || Int64.compare(frame.sequence, Int64.ofInt(0)) <= 0 ||
      Int64.compare(frame.sourceTimestampNs, Int64.ofInt(0)) < 0 ||
      frame.sourceClockId == null || frame.sourceClockId.length == 0 || frame.sourceClockId == "unspecified" ||
      Int64.compare(nowNs, Int64.ofInt(0)) < 0 || robotClockId == null ||
      robotClockId.length == 0 || robotClockId == "unspecified" ||
      Int64.compare(maximumAgeNs, Int64.ofInt(0)) <= 0)
      throw "Force/torque sample has the wrong sensor, frame, units contract, clock or age bound";

    var mapped:Null<MappedTimestamp> = frame.sourceClockId == robotClockId
      ? new MappedTimestamp(frame.sourceTimestampNs, Int64.ofInt(0))
      : mappings == null ? null : mappings.map(frame.sourceTimestampNs, frame.sourceClockId, robotClockId);
    if (mapped == null) throw 'Force/torque sensor "${frame.sensorId}" has no valid mapping to robot clock "$robotClockId"';
    var earliest = ClockMapping.checkedSubtract(mapped.valueNs, mapped.errorBoundNs);
    var latest = ClockMapping.checkedAdd(mapped.valueNs, mapped.errorBoundNs);
    var age = earliest == null ? null : ClockMapping.checkedSubtract(nowNs, earliest);
    if (earliest == null || latest == null || age == null || Int64.compare(latest, nowNs) > 0 ||
      Int64.compare(age, maximumAgeNs) > 0)
      throw 'Force/torque sensor "${frame.sensorId}" is stale or future-dated outside its clock uncertainty';

    var values = [for (index in 0...6) frame.values.get(index)];
    for (value in values) if (!Math.isFinite(value))
      throw 'Force/torque sensor "${frame.sensorId}" contains a non-finite SI value';
    return new ForceTorqueMeasurement(frame.sensorId, frame.frameId, frame.sequence,
      frame.sourceTimestampNs, frame.sourceClockId, mapped.valueNs, mapped.errorBoundNs,
      new Wrench3(new Vec3(values[0], values[1], values[2]),
        new Vec3(values[3], values[4], values[5])));
  }
}

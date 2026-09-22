package robotkit.world;

import haxe.Int64;

/** Transport-independent immutable observation of one logical robot. */
class RobotSnapshot {
  public final id:RobotId;
  public final sourceSequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final positions:ImmutableFloatArray;
  public final velocities:ImmutableFloatArray;
  public final efforts:ImmutableFloatArray;
  public final sensors:ImmutableSensorArray;
  public final mode:Int;
  public final faultCode:Int;

  /** Compatibility alias; new code should name the clock explicitly. */
  public var timestampNs(get, never):Int64;

  public function new(
    id:RobotId,
    sourceSequence:Int64,
    sourceTimestampNs:Int64,
    positions:Array<Float>,
    velocities:Array<Float>,
    efforts:Array<Float>,
    mode:Int,
    faultCode:Int,
    ?receivedTimestampNs:Int64,
    ?sensors:Array<SensorFrame>
  ) {
    this.id = id;
    this.sourceSequence = sourceSequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0)
      : receivedTimestampNs;
    this.positions = new ImmutableFloatArray(positions);
    this.velocities = new ImmutableFloatArray(velocities);
    this.efforts = new ImmutableFloatArray(efforts);
    this.sensors = new ImmutableSensorArray(sensors);
    this.mode = mode;
    this.faultCode = faultCode;
  }

  inline function get_timestampNs():Int64 return sourceTimestampNs;
}

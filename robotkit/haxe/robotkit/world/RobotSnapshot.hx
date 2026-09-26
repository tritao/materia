package robotkit.world;

import haxe.Int64;

/** Transport-independent immutable observation of one logical robot. */
class RobotSnapshot {
  public final id:RobotId;
  public final sourceSequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;
  public final positions:ImmutableFloatArray;
  public final velocities:ImmutableFloatArray;
  public final efforts:ImmutableFloatArray;
  public final sensors:ImmutableSensorArray;
  public final mode:Int;
  public final faultCode:Int;
  /** Current runtime safety state: ready, stopping, emergency-stop, or fault. */
  public final safety:Int;
  /** Number of timestamped trajectory points currently owned by the runtime. */
  public final trajectoryQueueDepth:Int;
  /** Whether the runtime is currently consuming a timestamped trajectory. */
  public final trajectoryActive:Bool;
  /** Runtime trajectory clock, in the runtime's owner timebase. */
  public final trajectoryTimeNs:Int64;
  /** Timestamp of the last point currently owned by the runtime. */
  public final trajectoryDurationNs:Int64;
  /** Stable identity of the chunk currently running or last stopped. */
  public final trajectoryTag:Int64;
  /** Time within trajectoryTag, in nanoseconds. */
  public final trajectoryTagTimeNs:Int64;

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
    ?sensors:Array<SensorFrame>,
    ?sourceClockId:String = "unspecified",
    ?receivedClockId:String = "robotkit.monotonic",
    ?safety:Int = 0,
    ?trajectoryQueueDepth:Int = 0,
    ?trajectoryActive:Bool = false,
    ?trajectoryTimeNs:Int64,
    ?trajectoryDurationNs:Int64,
    ?trajectoryTag:Int64,
    ?trajectoryTagTimeNs:Int64
  ) {
    this.id = id;
    this.sourceSequence = sourceSequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0)
      : receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
    this.positions = new ImmutableFloatArray(positions);
    this.velocities = new ImmutableFloatArray(velocities);
    this.efforts = new ImmutableFloatArray(efforts);
    this.sensors = new ImmutableSensorArray(sensors);
    this.mode = mode;
    this.faultCode = faultCode;
    this.safety = safety;
    this.trajectoryQueueDepth = trajectoryQueueDepth == null ? 0 : trajectoryQueueDepth;
    this.trajectoryActive = trajectoryActive == true;
    this.trajectoryTimeNs = trajectoryTimeNs == null ? Int64.ofInt(0) : trajectoryTimeNs;
    this.trajectoryDurationNs = trajectoryDurationNs == null
      ? Int64.ofInt(0) : trajectoryDurationNs;
    this.trajectoryTag = trajectoryTag == null ? Int64.ofInt(0) : trajectoryTag;
    this.trajectoryTagTimeNs = trajectoryTagTimeNs == null
      ? Int64.ofInt(0) : trajectoryTagTimeNs;
  }

  inline function get_timestampNs():Int64 return sourceTimestampNs;
}

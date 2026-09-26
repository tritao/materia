package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.world.ImmutableFloatArray;
import robotkit.world.ImmutableSensorArray;
import robotkit.world.SensorFrame;

/**
 * Immutable-by-ownership copy of one robot runtime observation.
 *
 * The arrays belong to this snapshot and are never reused by the runtime.
 * Consumers must treat them as read-only and retain the snapshot instead of
 * reaching back into mutable runtime state.
 */
class RobotSnapshot {
  public final robotId:Int64;
  public final sequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final mode:Int;
  public final safety:Int;
  public final endpoint:Int;
  public final faultCode:Int;
  public final q:ImmutableFloatArray;
  public final dq:ImmutableFloatArray;
  public final effort:ImmutableFloatArray;
  public final sensors:ImmutableSensorArray;
  public final trajectoryQueueDepth:Int;
  public final trajectoryActive:Bool;
  public final trajectoryTimeNs:Int64;
  public final trajectoryDurationNs:Int64;
  public final trajectoryTag:Int64;
  public final trajectoryTagTimeNs:Int64;

  /** Compatibility alias; source time is the runtime's primary observation clock. */
  public var timestampNs(get, never):Int64;

  public function new(robotId:Int64, sequence:Int64, sourceTimestampNs:Int64,
      mode:Int, safety:Int, endpoint:Int, faultCode:Int,
      q:Array<Float>, dq:Array<Float>, effort:Array<Float>,
      ?receivedTimestampNs:Int64, ?sensors:Array<SensorFrame>,
      ?trajectoryQueueDepth:Int, ?trajectoryActive:Bool,
      ?trajectoryTimeNs:Int64, ?trajectoryDurationNs:Int64,
      ?trajectoryTag:Int64, ?trajectoryTagTimeNs:Int64) {
    this.robotId = robotId;
    this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0)
      : receivedTimestampNs;
    this.mode = mode;
    this.safety = safety;
    this.endpoint = endpoint;
    this.faultCode = faultCode;
    this.q = new ImmutableFloatArray(q);
    this.dq = new ImmutableFloatArray(dq);
    this.effort = new ImmutableFloatArray(effort);
    this.sensors = new ImmutableSensorArray(sensors);
    this.trajectoryQueueDepth = trajectoryQueueDepth == null ? 0 : trajectoryQueueDepth;
    this.trajectoryActive = trajectoryActive == true;
    this.trajectoryTimeNs = trajectoryTimeNs == null ? Int64.ofInt(0) : trajectoryTimeNs;
    this.trajectoryDurationNs = trajectoryDurationNs == null
      ? Int64.ofInt(0) : trajectoryDurationNs;
    this.trajectoryTag = trajectoryTag == null ? Int64.ofInt(0) : trajectoryTag;
    this.trajectoryTagTimeNs = trajectoryTagTimeNs == null
      ? Int64.ofInt(0) : trajectoryTagTimeNs;
  }

  /** Converts the native ABI value while leaving the semantic robot ID unset. */
  @:allow(RobotRuntime)
  static function fromNative(value:rk_robot_snapshot, layout:Array<RobotRuntimeSensorBlueprint>):RobotSnapshot {
    var positions:Array<Float> = [];
    var velocities:Array<Float> = [];
    var efforts:Array<Float> = [];
    var count = value.get_joint_count();
    for (index in 0...count) {
      positions.push(value.get_position(index));
      velocities.push(value.get_velocity(index));
      efforts.push(value.get_effort(index));
    }
    var frames:Array<SensorFrame> = [];
    if (value.get_sensor_count() > layout.length)
      throw "Runtime published more sensor slots than its compiled blueprint";
    for (i in 0...value.get_sensor_count()) {
      var sample = value.get_sensors(i);
      if (sample.get_sequence() == Int64.ofInt(0)) continue;
      var config = layout[i];
      frames.push(new SensorFrame(config.id, config.kind, config.frameId,
        sample.get_sequence(), sample.get_source_timestamp_ns(),
        [for (j in 0...sample.get_value_count()) sample.get_values(j)], sample.get_received_timestamp_ns(),
        config.linkId, config.position.toArray(), config.rotation.toArray()));
    }
    // Standalone endpoints may only report joint state; expose configured
    // encoders from that actual state, never synthesize other sensor kinds.
    if (value.get_sensor_count() == 0 && value.get_sequence() != Int64.ofInt(0))
      for (config in layout) if (config.kind == "joint_encoder")
        frames.push(new SensorFrame(config.id, config.kind, config.frameId,
          value.get_sequence(), value.get_source_timestamp_ns(), positions, value.get_received_timestamp_ns(),
          config.linkId, config.position.toArray(), config.rotation.toArray()));
    return new RobotSnapshot(Int64.ofInt(0), value.get_sequence(), value.get_source_timestamp_ns(),
      value.get_mode(), value.get_safety(), value.get_endpoint(), value.get_fault_code(),
      positions, velocities, efforts, value.get_received_timestamp_ns(), frames,
      value.get_trajectory_queue_depth(), value.get_trajectory_active() != 0,
      value.get_trajectory_time_ns(), value.get_trajectory_duration_ns(),
      value.get_trajectory_tag(), value.get_trajectory_tag_time_ns());
  }

  /** Returns an immutable copy associated with a caller-provided robot ID. */
  public function withRobotId(value:Int64):RobotSnapshot
    return new RobotSnapshot(value, sequence, sourceTimestampNs, mode, safety, endpoint, faultCode,
      q.toArray(), dq.toArray(), effort.toArray(), receivedTimestampNs, sensors.toArray(),
      trajectoryQueueDepth, trajectoryActive, trajectoryTimeNs, trajectoryDurationNs,
      trajectoryTag, trajectoryTagTimeNs);

  inline function get_timestampNs():Int64 return sourceTimestampNs;
}

package robotkit.world;

import robotkit.runtime.RobotSnapshot;

/** Shared projection of measured runtime state, never synthesized sensor data. */
class RobotSensorFrames {
  public static function fromRuntimeSnapshot(snapshot:RobotSnapshot):Array<SensorFrame> {
    var frames = [
      new SensorFrame("joint_encoders", "joint_encoder", "base_link", snapshot.sequence,
        snapshot.sourceTimestampNs, snapshot.q.toArray(), snapshot.receivedTimestampNs)
    ];
    if (snapshot.imu.length == 6)
      frames.push(new SensorFrame("imu", "imu", "base_link", snapshot.sequence,
        snapshot.sourceTimestampNs, snapshot.imu.toArray(), snapshot.receivedTimestampNs));
    if (snapshot.lidar.length == 8)
      frames.push(new SensorFrame("lidar", "lidar", "base_link", snapshot.sequence,
        snapshot.sourceTimestampNs, snapshot.lidar.toArray(), snapshot.receivedTimestampNs));
    return frames;
  }
}

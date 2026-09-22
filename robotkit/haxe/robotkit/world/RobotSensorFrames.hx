package robotkit.world;

import robotkit.runtime.RobotSnapshot;

/**
 * Canonical first sensor projection shared by simulation and robotd.
 *
 * The native SensorKit adapters can replace the model later without changing
 * RobotWorld, RobotClient, or behavior code. Keeping encoder/IMU/LiDAR
 * identity and timestamp rules in one place prevents simulated and remote
 * robots from drifting apart while backend sensor models are integrated.
 */
class RobotSensorFrames {
  public static function fromRuntimeSnapshot(snapshot:RobotSnapshot):Array<SensorFrame> {
    return [
      new SensorFrame("joint_encoders", "joint_encoder", "base_link", snapshot.sequence,
        snapshot.sourceTimestampNs, snapshot.q.toArray(), snapshot.receivedTimestampNs),
      new SensorFrame("imu", "imu", "base_link", snapshot.sequence,
        snapshot.sourceTimestampNs,
        [snapshot.q.length > 0 ? snapshot.q.get(0) : 0.0,
         snapshot.dq.length > 0 ? snapshot.dq.get(0) : 0.0,
         0.0, 0.0, 0.0, 9.81],
        snapshot.receivedTimestampNs),
      new SensorFrame("lidar", "lidar", "base_link", snapshot.sequence,
        snapshot.sourceTimestampNs,
        [for (_ in 0...8) 10.0], snapshot.receivedTimestampNs)
    ];
  }
}

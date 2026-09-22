package robotkit.world;

import robotkit.runtime.RobotSnapshot;

/** Shared projection of measured runtime state, never synthesized sensor data. */
class RobotSensorFrames {
  public static function fromRuntimeSnapshot(snapshot:RobotSnapshot):Array<SensorFrame> {
    return snapshot.sensors.toArray();
  }
}

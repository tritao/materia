package robotkit.perception;



import robotkit.mobile.Pose2;
import robotkit.navigation.OccupancyGrid2;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.core.SensorFrame;

/**
 * Drops the planar LiDAR returns that a known map already explains, so what perception reports is
 * only what the map lacks: a return whose point lies on (or within `toleranceMeters` of) an occupied
 * cell becomes a ray with no return. Without it a scan of a walled room reads as obstacles all
 * round the robot.
 */
class LidarMapFilter implements ScanFilter {
  public final grid:OccupancyGrid2;
  public final toleranceMeters:Float;
  public final maxRangeMeters:Float;
  public final startAngleRadians:Float;
  public final fieldOfViewRadians:Float;

  /** Builds a filter for a LiDAR compiled from a robot model. */
  public static function fromSensor(grid:OccupancyGrid2, sensor:RobotRuntimeSensorBlueprint,
      toleranceMeters:Float):LidarMapFilter {
    if (sensor == null || sensor.kind != "lidar") throw "LiDAR map filter requires a compiled LiDAR sensor";
    return new LidarMapFilter(grid, toleranceMeters, sensor.maxRange, sensor.startAngleRadians,
      sensor.fieldOfViewRadians);
  }

  public function new(grid:OccupancyGrid2, toleranceMeters:Float, maxRangeMeters:Float,
      ?startAngleRadians:Float = 0.0, ?fieldOfViewRadians:Float = Math.PI * 2.0) {
    if (grid == null || !Math.isFinite(toleranceMeters) || toleranceMeters < 0.0 ||
        !Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 || !Math.isFinite(startAngleRadians) ||
        !Math.isFinite(fieldOfViewRadians) || fieldOfViewRadians <= 0.0)
      throw "LiDAR map filter requires a grid, a nonnegative tolerance, and a finite scan layout";
    this.grid = grid;
    this.toleranceMeters = toleranceMeters;
    this.maxRangeMeters = maxRangeMeters;
    this.startAngleRadians = startAngleRadians;
    this.fieldOfViewRadians = fieldOfViewRadians;
  }

  public function applies(frame:SensorFrame):Bool return frame.kind == "lidar";

  public function filter(frame:SensorFrame, referenceFromSensor:Pose2):SensorFrame {
    var rays = frame.values.length;
    var values = frame.values.toArray();
    var explained = 0;
    for (index in 0...rays) {
      var range = values[index];
      if (!Math.isFinite(range) || range >= maxRangeMeters) continue;
      var bearing = LidarObstaclePerception.bearing(index, rays, startAngleRadians, fieldOfViewRadians);
      var hit = referenceFromSensor.compose(new Pose2(range * Math.cos(bearing), range * Math.sin(bearing)));
      if (grid.occupiedWithin(hit.x, hit.y, toleranceMeters)) {
        values[index] = maxRangeMeters;
        explained++;
      }
    }
    if (explained == 0) return frame;
    return new SensorFrame(frame.sensorId, frame.kind, frame.frameId, frame.sequence, frame.sourceTimestampNs,
      values, frame.receivedTimestampNs, frame.linkId, frame.mountPosition.toArray(), frame.mountRotation.toArray(),
      frame.sourceClockId, frame.receivedClockId, frame.image);
  }
}

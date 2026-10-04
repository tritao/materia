package robotkit.perception;

import robotkit.mobile.Pose2;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.core.SensorFrame;

/**
 * What a planar LiDAR's scans say about free space. A scan sees past a disk when every ray that would
 * cross it returned beyond it (or nothing within range), so an obstacle remembered there can be forgotten.
 */
class LidarFreeSpace {
  public final maxRangeMeters:Float;
  public final minRangeMeters:Float;
  public final startAngleRadians:Float;
  public final fieldOfViewRadians:Float;

  public static function fromSensor(sensor:RobotRuntimeSensorBlueprint, ?minRangeMeters:Float = 0.05):LidarFreeSpace {
    if (sensor == null || sensor.kind != "lidar") throw "LiDAR free space requires a compiled LiDAR sensor";
    return new LidarFreeSpace(sensor.maxRange, minRangeMeters, sensor.startAngleRadians, sensor.fieldOfViewRadians);
  }

  public function new(maxRangeMeters:Float, ?minRangeMeters:Float = 0.05, ?startAngleRadians:Float = 0.0,
      ?fieldOfViewRadians:Float = Math.PI * 2.0) {
    if (!Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 || !Math.isFinite(minRangeMeters) ||
        minRangeMeters < 0.0 || !Math.isFinite(startAngleRadians) || !Math.isFinite(fieldOfViewRadians) ||
        fieldOfViewRadians <= 0.0)
      throw "LiDAR free space requires a finite scan layout";
    this.maxRangeMeters = maxRangeMeters;
    this.minRangeMeters = minRangeMeters;
    this.startAngleRadians = startAngleRadians;
    this.fieldOfViewRadians = fieldOfViewRadians;
  }

  /** The free space one scan shows, its sensor standing at `referenceFromSensor`. */
  public function viewing(frame:SensorFrame, referenceFromSensor:Pose2):FreeSpaceView
    return new LidarScanView(this, frame.values.toArray(), referenceFromSensor);
}

private class LidarScanView implements FreeSpaceView {
  final layout:LidarFreeSpace;
  final ranges:Array<Float>;
  final sensor:Pose2;

  public function new(layout:LidarFreeSpace, ranges:Array<Float>, sensor:Pose2) {
    this.layout = layout;
    this.ranges = ranges;
    this.sensor = sensor;
  }

  public function freeAt(x:Float, y:Float, radiusMeters:Float):Bool {
    var rays = ranges.length;
    if (rays == 0) return false;
    var local = new Pose2(x, y).relativeTo(sensor);
    var distance = Math.sqrt(local.x * local.x + local.y * local.y);
    // A place the sensor sits in, or reaches only partly, says nothing.
    if (distance <= radiusMeters || distance + radiusMeters > layout.maxRangeMeters) return false;
    var bearing = Math.atan2(local.y, local.x);
    var spread = Math.asin(Math.min(1.0, radiusMeters / distance));
    var step = LidarObstaclePerception.bearing(1, rays, 0.0, layout.fieldOfViewRadians);
    var first = Math.ceil(Pose2.wrapAngle(bearing - spread - layout.startAngleRadians) / step - 1e-9);
    var count = Std.int(Math.max(1.0, Math.floor(2.0 * spread / step + 1e-9) + 1.0));
    var full = Math.abs(layout.fieldOfViewRadians - Math.PI * 2.0) <= 1e-6;
    if (first < 0 && full) first += rays;
    var checked = 0;
    for (offset in 0...count) {
      var index = Std.int(first) + offset;
      if (full) index = index % rays;
      else if (index < 0 || index >= rays) return false;
      var theta = LidarObstaclePerception.bearing(index, rays, layout.startAngleRadians, layout.fieldOfViewRadians) - bearing;
      var across = distance * Math.sin(theta);
      if (Math.abs(across) >= radiusMeters) continue;
      var exit = distance * Math.cos(theta) + Math.sqrt(radiusMeters * radiusMeters - across * across);
      var range = ranges[index];
      if (!Math.isFinite(range) || range <= layout.minRangeMeters) return false;
      if (range < layout.maxRangeMeters && range <= exit) return false;
      checked++;
    }
    // No ray crossed the disk: it fell between two, and nothing is known of it.
    return checked > 0;
  }
}

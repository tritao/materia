package robotkit.perception;

import robotkit.mobile.Pose2;
import robotkit.world.SensorFrame;

/** Converts planar LiDAR ranges into circular obstacle observations in the sensor frame. */
class LidarObstaclePerception implements Perception {
  public final maxRangeMeters:Float;
  public final minRangeMeters:Float;
  public final obstacleRadiusMeters:Float;
  public final minConfidence:Float;

  public function new(maxRangeMeters:Float, obstacleRadiusMeters:Float,
      ?minRangeMeters:Float = 0.05, ?minConfidence:Float = 0.5) {
    if (!Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 ||
        !Math.isFinite(obstacleRadiusMeters) || obstacleRadiusMeters <= 0.0 ||
        !Math.isFinite(minRangeMeters) || minRangeMeters < 0.0 ||
        minRangeMeters >= maxRangeMeters || !Math.isFinite(minConfidence) ||
        minConfidence < 0.0 || minConfidence > 1.0)
      throw "LiDAR perception configuration is invalid";
    this.maxRangeMeters = maxRangeMeters;
    this.obstacleRadiusMeters = obstacleRadiusMeters;
    this.minRangeMeters = minRangeMeters;
    this.minConfidence = minConfidence;
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Perception requires sensor frames";
    var detections:Array<Detection> = [];
    var obstacles:Array<Obstacle> = [];
    for (frame in frames) {
      if (frame == null || frame.kind != "lidar" || frame.values.length == 0) continue;
      var rays = frame.values.length;
      for (index in 0...rays) {
        var range = frame.values.get(index);
        if (!Math.isFinite(range) || range <= minRangeMeters || range >= maxRangeMeters)
          continue;
        var angle = Math.PI * 2.0 * index / rays;
        var detection = new Detection(
          '${frame.sensorId}:${Std.string(frame.sequence)}:$index', "obstacle", minConfidence,
          new Pose2(range * Math.cos(angle), range * Math.sin(angle), 0.0),
          frame.frameId, frame.sequence, frame.sourceTimestampNs,
          frame.receivedTimestampNs, frame.sourceClockId, frame.receivedClockId);
        detections.push(detection);
        obstacles.push(new Obstacle(detection, obstacleRadiusMeters));
      }
    }
    return new PerceptionSnapshot(detections, obstacles);
  }
}

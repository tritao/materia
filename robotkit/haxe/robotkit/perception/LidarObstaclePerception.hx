package robotkit.perception;

import robotkit.mobile.Pose2;
import robotkit.world.SensorFrame;

/** Groups adjacent planar LiDAR returns into circular obstacle observations. */
class LidarObstaclePerception implements Perception {
  public final maxRangeMeters:Float;
  public final minRangeMeters:Float;
  public final obstacleRadiusMeters:Float;
  public final minConfidence:Float;
  public final maxClusterGapMeters:Float;
  public final startAngleRadians:Float;
  public final fieldOfViewRadians:Float;

  public function new(maxRangeMeters:Float, obstacleRadiusMeters:Float,
      ?minRangeMeters:Float = 0.05, ?minConfidence:Float = 0.5,
      ?maxClusterGapMeters:Float = 0.1, ?startAngleRadians:Float = 0.0,
      ?fieldOfViewRadians:Float = Math.PI * 2.0) {
    if (!Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 ||
        !Math.isFinite(obstacleRadiusMeters) || obstacleRadiusMeters <= 0.0 ||
        !Math.isFinite(minRangeMeters) || minRangeMeters < 0.0 ||
        minRangeMeters >= maxRangeMeters || !Math.isFinite(minConfidence) ||
        minConfidence < 0.0 || minConfidence > 1.0 ||
        !Math.isFinite(maxClusterGapMeters) || maxClusterGapMeters < 0.0 ||
        !Math.isFinite(startAngleRadians) || !Math.isFinite(fieldOfViewRadians) ||
        fieldOfViewRadians <= 0.0 || fieldOfViewRadians > Math.PI * 2.0 + 1e-6)
      throw "LiDAR perception configuration is invalid";
    this.maxRangeMeters = maxRangeMeters;
    this.obstacleRadiusMeters = obstacleRadiusMeters;
    this.minRangeMeters = minRangeMeters;
    this.minConfidence = minConfidence;
    this.maxClusterGapMeters = maxClusterGapMeters;
    this.startAngleRadians = startAngleRadians;
    this.fieldOfViewRadians = fieldOfViewRadians;
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Perception requires sensor frames";
    var detections:Array<Detection> = [];
    var obstacles:Array<Obstacle> = [];
    for (frame in frames) {
      if (frame == null || frame.kind != "lidar" || frame.values.length == 0) continue;
      var rays = frame.values.length;
      var fullCircle = Math.abs(fieldOfViewRadians - Math.PI * 2.0) <= 1e-6;
      var angleIncrement = fullCircle
        ? fieldOfViewRadians / rays
        : (rays <= 1 ? 0.0 : fieldOfViewRadians / (rays - 1));
      var clusters:Array<Array<LidarPoint>> = [];
      var active:Array<LidarPoint> = [];
      for (index in 0...rays) {
        var range = frame.values.get(index);
        if (!Math.isFinite(range) || range <= minRangeMeters || range >= maxRangeMeters) {
          if (active.length > 0) clusters.push(active);
          active = [];
          continue;
        }
        var angle = startAngleRadians + angleIncrement * index;
        var point = new LidarPoint(index, range * Math.cos(angle),
          range * Math.sin(angle), range);
        if (active.length > 0 && !connects(active[active.length - 1], point,
            angleIncrement)) {
          clusters.push(active);
          active = [];
        }
        active.push(point);
      }
      if (active.length > 0) clusters.push(active);

      // A complete scan has adjacent first and last rays. Join them only if both
      // edge rays were observed and their measured points are spatially close.
      if (fullCircle && clusters.length > 1) {
        var first = clusters[0];
        var last = clusters[clusters.length - 1];
        if (first[0].rayIndex == 0 && last[last.length - 1].rayIndex == rays - 1 &&
            connects(last[last.length - 1], first[0], angleIncrement)) {
          var merged = last.concat(first);
          clusters[0] = merged;
          clusters.pop();
        }
      }

      for (clusterIndex in 0...clusters.length) {
        var cluster = clusters[clusterIndex];
        var centerX = 0.0;
        var centerY = 0.0;
        for (point in cluster) {
          centerX += point.x;
          centerY += point.y;
        }
        centerX /= cluster.length;
        centerY /= cluster.length;
        var extent = 0.0;
        for (point in cluster) {
          var dx = point.x - centerX;
          var dy = point.y - centerY;
          extent = Math.max(extent, Math.pow(dx * dx + dy * dy, 0.5));
        }
        var confidence = Math.min(1.0,
          minConfidence + (cluster.length - 1) * 0.03);
        var detection = new Detection(
          '${frame.sensorId}:${Std.string(frame.sequence)}:$clusterIndex', "obstacle",
          confidence, new Pose2(centerX, centerY, 0.0), frame.frameId,
          frame.sequence, frame.sourceTimestampNs, frame.receivedTimestampNs,
          frame.sourceClockId, frame.receivedClockId);
        detections.push(detection);
        obstacles.push(new Obstacle(detection,
          Math.max(obstacleRadiusMeters, extent + obstacleRadiusMeters)));
      }
    }
    return new PerceptionSnapshot(detections, obstacles);
  }

  function connects(from:LidarPoint, to:LidarPoint, angularStep:Float):Bool {
    var dx = to.x - from.x;
    var dy = to.y - from.y;
    var distance = Math.pow(dx * dx + dy * dy, 0.5);
    var range = Math.max(from.rangeMeters, to.rangeMeters);
    var gapLimit = maxClusterGapMeters + range * Math.abs(angularStep) * 1.5;
    return distance <= gapLimit;
  }
}

private class LidarPoint {
  public final rayIndex:Int;
  public final x:Float;
  public final y:Float;
  public final rangeMeters:Float;

  public function new(rayIndex:Int, x:Float, y:Float, rangeMeters:Float) {
    this.rayIndex = rayIndex;
    this.x = x;
    this.y = y;
    this.rangeMeters = rangeMeters;
  }
}

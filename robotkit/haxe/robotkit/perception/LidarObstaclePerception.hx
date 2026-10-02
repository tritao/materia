package robotkit.perception;

import robotkit.mobile.Pose2;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
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
  /** How far (m) hits may stray from a straight run before it is split into two obstacles. */
  public final segmentToleranceMeters:Float;

  /** Builds a scan processor from the LiDAR settings compiled from a robot model. */
  public static function fromSensor(sensor:RobotRuntimeSensorBlueprint,
      obstacleRadiusMeters:Float, ?minRangeMeters:Float = 0.05,
      ?minConfidence:Float = 0.5, ?maxClusterGapMeters:Float = 0.1):LidarObstaclePerception {
    if (sensor == null || sensor.kind != "lidar")
      throw "LiDAR perception requires a compiled LiDAR sensor";
    return new LidarObstaclePerception(sensor.maxRange, obstacleRadiusMeters,
      minRangeMeters, minConfidence, maxClusterGapMeters,
      sensor.startAngleRadians, sensor.fieldOfViewRadians);
  }

  /** Builds perception from a sensor selected by its stable authored ID. */
  public static function fromBlueprint(blueprint:RobotRuntimeBlueprint,
      sensorId:String, obstacleRadiusMeters:Float,
      ?minRangeMeters:Float = 0.05, ?minConfidence:Float = 0.5,
      ?maxClusterGapMeters:Float = 0.1):LidarObstaclePerception {
    if (blueprint == null) throw "LiDAR perception requires a compiled robot blueprint";
    var sensor = blueprint.sensorById(sensorId);
    if (sensor == null)
      throw 'Compiled robot has no sensor with ID "$sensorId"';
    return fromSensor(sensor, obstacleRadiusMeters, minRangeMeters,
      minConfidence, maxClusterGapMeters);
  }

  public function new(maxRangeMeters:Float, obstacleRadiusMeters:Float,
      ?minRangeMeters:Float = 0.05, ?minConfidence:Float = 0.5,
      ?maxClusterGapMeters:Float = 0.1, ?startAngleRadians:Float = 0.0,
      ?fieldOfViewRadians:Float = Math.PI * 2.0, ?segmentToleranceMeters:Float = 0.05) {
    if (!Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 ||
        !Math.isFinite(obstacleRadiusMeters) || obstacleRadiusMeters <= 0.0 ||
        !Math.isFinite(minRangeMeters) || minRangeMeters < 0.0 ||
        minRangeMeters >= maxRangeMeters || !Math.isFinite(minConfidence) ||
        minConfidence < 0.0 || minConfidence > 1.0 ||
        !Math.isFinite(maxClusterGapMeters) || maxClusterGapMeters < 0.0 ||
        !Math.isFinite(startAngleRadians) || !Math.isFinite(fieldOfViewRadians) ||
        fieldOfViewRadians <= 0.0 || fieldOfViewRadians > Math.PI * 2.0 + 1e-6 ||
        !Math.isFinite(segmentToleranceMeters) || segmentToleranceMeters < 0.0)
      throw "LiDAR perception configuration is invalid";
    this.maxRangeMeters = maxRangeMeters;
    this.obstacleRadiusMeters = obstacleRadiusMeters;
    this.minRangeMeters = minRangeMeters;
    this.minConfidence = minConfidence;
    this.maxClusterGapMeters = maxClusterGapMeters;
    this.startAngleRadians = startAngleRadians;
    this.fieldOfViewRadians = fieldOfViewRadians;
    this.segmentToleranceMeters = segmentToleranceMeters;
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Perception requires sensor frames";
    var detections:Array<Detection> = [];
    var obstacles:Array<Obstacle> = [];
    for (frame in frames) {
      if (frame == null || frame.kind != "lidar" || frame.values.length == 0) continue;
      var rays = frame.values.length;
      var fullCircle = Math.abs(fieldOfViewRadians - Math.PI * 2.0) <= 1e-6;
      var angleIncrement = increment(rays, fieldOfViewRadians);
      var clusters:Array<Array<LidarPoint>> = [];
      var active:Array<LidarPoint> = [];
      for (index in 0...rays) {
        var range = frame.values.get(index);
        if (!Math.isFinite(range) || range <= minRangeMeters || range >= maxRangeMeters) {
          if (active.length > 0) clusters.push(active);
          active = [];
          continue;
        }
        var angle = bearing(index, rays, startAngleRadians, fieldOfViewRadians);
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
        var confidence = Math.min(1.0,
          minConfidence + (cluster.length - 1) * 0.03);
        function detect(id:String, pose:Pose2):Detection
          return new Detection(id, "obstacle", confidence, pose, frame.frameId,
            frame.sequence, frame.sourceTimestampNs, frame.receivedTimestampNs,
            frame.sourceClockId, frame.receivedClockId);
        var baseId = '${frame.sensorId}:${Std.string(frame.sequence)}:$clusterIndex';
        detections.push(detect(baseId, new Pose2(centerX, centerY, 0.0)));
        // The obstacle follows the hits along the visible surface: a capsule per straight run of them, so a
        // flat wall is a thin segment, a corner two, and a lone return a disk.
        var corners = simplify(cluster, 0, cluster.length - 1);
        if (corners.length == 1) {
          obstacles.push(new Obstacle(detect(baseId, new Pose2(cluster[0].x, cluster[0].y, 0.0)), obstacleRadiusMeters));
        } else {
          for (index in 1...corners.length) {
            var from = cluster[corners[index - 1]], to = cluster[corners[index]];
            var dx = to.x - from.x, dy = to.y - from.y;
            var length = Math.sqrt(dx * dx + dy * dy);
            var mid = new Pose2((from.x + to.x) * 0.5, (from.y + to.y) * 0.5, Math.atan2(dy, dx));
            obstacles.push(new Obstacle(detect(corners.length == 2 ? baseId : '$baseId.${index - 1}', mid),
              obstacleRadiusMeters, length * 0.5));
          }
        }
      }
    }
    return new PerceptionSnapshot(detections, obstacles);
  }

  /**
   * Indices of the hits at the ends of the straight runs `points[first..last]` follow, to within
   * `segmentToleranceMeters` (Douglas-Peucker): the ends, plus wherever the run bends.
   */
  function simplify(points:Array<LidarPoint>, first:Int, last:Int):Array<Int> {
    if (last <= first) return [first];
    var a = points[first], b = points[last];
    var dx = b.x - a.x, dy = b.y - a.y;
    var length = Math.sqrt(dx * dx + dy * dy);
    var farthest = -1, farthestDistance = 0.0;
    for (index in first + 1...last) {
      var p = points[index];
      var distance = length <= 1e-9 ? Math.sqrt((p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y))
        : Math.abs(dx * (p.y - a.y) - dy * (p.x - a.x)) / length;
      if (distance > farthestDistance) { farthest = index; farthestDistance = distance; }
    }
    if (farthest < 0 || farthestDistance <= segmentToleranceMeters) return [first, last];
    var left = simplify(points, first, farthest), right = simplify(points, farthest, last);
    return left.concat(right.slice(1));
  }

  /** Angle between adjacent rays of a scan of `rays` returns over `fieldOfViewRadians`. */
  static function increment(rays:Int, fieldOfViewRadians:Float):Float {
    var fullCircle = Math.abs(fieldOfViewRadians - Math.PI * 2.0) <= 1e-6;
    return fullCircle
      ? fieldOfViewRadians / rays
      : (rays <= 1 ? 0.0 : fieldOfViewRadians / (rays - 1));
  }

  /** Bearing of ray `index` in the sensor frame, as the simulated sensor lays its rays out. */
  public static function bearing(index:Int, rays:Int, startAngleRadians:Float,
      fieldOfViewRadians:Float):Float
    return startAngleRadians + increment(rays, fieldOfViewRadians) * index;

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

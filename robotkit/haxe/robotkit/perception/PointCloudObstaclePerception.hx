package robotkit.perception;

import robotkit.mobile.Pose2;
import robotkit.world.SensorFrame;

/** Extracts planar obstacle clusters from XYZ point-cloud sensor frames. */
class PointCloudObstaclePerception implements Perception {
  public final maxRangeMeters:Float;
  public final minRangeMeters:Float;
  public final minHeightMeters:Float;
  public final maxHeightMeters:Float;
  public final obstacleRadiusMeters:Float;
  public final minConfidence:Float;
  public final clusterDistanceMeters:Float;
  public final minClusterPoints:Int;
  public final maxPointCount:Int;
  public final sensorKind:String;

  public function new(maxRangeMeters:Float, obstacleRadiusMeters:Float,
      ?minRangeMeters:Float = 0.05, ?minConfidence:Float = 0.85,
      ?clusterDistanceMeters:Float = 0.25,
      ?minHeightMeters:Float = -0.25, ?maxHeightMeters:Float = 2.5,
      ?minClusterPoints:Int = 3, ?maxPointCount:Int = 100000,
      ?sensorKind:String = "point_cloud") {
    if (!Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 ||
        !Math.isFinite(obstacleRadiusMeters) || obstacleRadiusMeters <= 0.0 ||
        !Math.isFinite(minRangeMeters) || minRangeMeters < 0.0 ||
        minRangeMeters >= maxRangeMeters || !Math.isFinite(minConfidence) ||
        minConfidence < 0.0 || minConfidence > 1.0 ||
        !Math.isFinite(clusterDistanceMeters) || clusterDistanceMeters <= 0.0 ||
        !Math.isFinite(minHeightMeters) || !Math.isFinite(maxHeightMeters) ||
        minHeightMeters >= maxHeightMeters || minClusterPoints < 1 ||
        maxPointCount < minClusterPoints || sensorKind == null || sensorKind.length == 0)
      throw "Point-cloud perception configuration is invalid";
    this.maxRangeMeters = maxRangeMeters;
    this.obstacleRadiusMeters = obstacleRadiusMeters;
    this.minRangeMeters = minRangeMeters;
    this.minConfidence = minConfidence;
    this.clusterDistanceMeters = clusterDistanceMeters;
    this.minHeightMeters = minHeightMeters;
    this.maxHeightMeters = maxHeightMeters;
    this.minClusterPoints = minClusterPoints;
    this.maxPointCount = maxPointCount;
    this.sensorKind = sensorKind;
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Perception requires sensor frames";
    var detections:Array<Detection> = [];
    var obstacles:Array<Obstacle> = [];
    for (frame in frames) {
      if (frame == null || frame.kind != sensorKind || frame.values.length == 0) continue;
      if (frame.values.length % 3 != 0)
        throw 'Point-cloud sensor "${frame.sensorId}" must provide XYZ triples';
      var count = Std.int(frame.values.length / 3);
      if (count > maxPointCount)
        throw 'Point-cloud sensor "${frame.sensorId}" exceeds the configured point limit';
      var points:Array<CloudPoint> = [];
      var cellSize = clusterDistanceMeters;
      var buckets = new Map<String,Array<Int>>();
      for (index in 0...count) {
        var x = frame.values.get(index * 3);
        var y = frame.values.get(index * 3 + 1);
        var z = frame.values.get(index * 3 + 2);
        if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z) ||
            z < minHeightMeters || z > maxHeightMeters) continue;
        var rangeSquared = x * x + y * y + z * z;
        if (rangeSquared <= minRangeMeters * minRangeMeters ||
            rangeSquared >= maxRangeMeters * maxRangeMeters) continue;
        var point = new CloudPoint(index, x, y);
        var pointIndex = points.length;
        points.push(point);
        var key = bucketKey(cellSize, x, y);
        var bucket = buckets.get(key);
        if (bucket == null) {
          bucket = [];
          buckets.set(key, bucket);
        }
        bucket.push(pointIndex);
      }

      var visited:Array<Bool> = [for (_ in 0...points.length) false];
      var distanceSquared = clusterDistanceMeters * clusterDistanceMeters;
      for (seed in 0...points.length) {
        if (visited[seed]) continue;
        var queue:Array<Int> = [seed];
        var cluster:Array<Int> = [];
        visited[seed] = true;
        var cursor = 0;
        while (cursor < queue.length) {
          var currentIndex = queue[cursor++];
          cluster.push(currentIndex);
          var current = points[currentIndex];
          var cellX = Std.int(Math.floor(current.x / cellSize));
          var cellY = Std.int(Math.floor(current.y / cellSize));
          for (neighborX in (cellX - 1)...(cellX + 2))
            for (neighborY in (cellY - 1)...(cellY + 2)) {
              var neighbors = buckets.get('$neighborX,$neighborY');
              if (neighbors == null) continue;
              for (neighborIndex in neighbors) {
                if (visited[neighborIndex]) continue;
                var neighbor = points[neighborIndex];
                var dx = current.x - neighbor.x;
                var dy = current.y - neighbor.y;
                if (dx * dx + dy * dy <= distanceSquared) {
                  visited[neighborIndex] = true;
                  queue.push(neighborIndex);
                }
              }
            }
        }
        if (cluster.length < minClusterPoints) continue;
        var centerX = 0.0;
        var centerY = 0.0;
        var firstIndex = count;
        for (pointIndex in cluster) {
          var point = points[pointIndex];
          centerX += point.x;
          centerY += point.y;
          if (point.sourceIndex < firstIndex) firstIndex = point.sourceIndex;
        }
        centerX /= cluster.length;
        centerY /= cluster.length;
        var extent = 0.0;
        for (pointIndex in cluster) {
          var point = points[pointIndex];
          var dx = point.x - centerX;
          var dy = point.y - centerY;
          var distance = Math.pow(dx * dx + dy * dy, 0.5);
          if (distance > extent) extent = distance;
        }
        var point = new Pose2(centerX, centerY);
        var outputFrame = frame.frameId;
        var detection = new Detection(
          '${frame.sensorId}:${Std.string(frame.sequence)}:$firstIndex',
          "obstacle", minConfidence, point, outputFrame, frame.sequence,
          frame.sourceTimestampNs, frame.receivedTimestampNs,
          frame.sourceClockId, frame.receivedClockId);
        detections.push(detection);
        obstacles.push(new Obstacle(detection, obstacleRadiusMeters + extent));
      }
    }
    return new PerceptionSnapshot(detections, obstacles);
  }

  static function bucketKey(cellSize:Float, x:Float, y:Float):String
    return '${Std.int(Math.floor(x / cellSize))},${Std.int(Math.floor(y / cellSize))}';
}

private class CloudPoint {
  public final sourceIndex:Int;
  public final x:Float;
  public final y:Float;

  public function new(sourceIndex:Int, x:Float, y:Float) {
    this.sourceIndex = sourceIndex;
    this.x = x;
    this.y = y;
  }
}

package robotkit.perception;

import robotkit.model.RobotModel;
import robotkit.world.SensorFrame;

/**
 * Unprojects calibrated depth images into a mounted 3D point cloud, then
 * clusters the points into planar obstacle observations.
 */
class DepthCameraObstaclePerception implements Perception {
  public final sensorId:String;
  public final sensorFrameId:String;
  public final bodyLinkId:String;
  public final intrinsics:PinholeCameraIntrinsics;
  public final pointCloudPerception:PointCloudObstaclePerception;
  public final minRangeMeters:Float;
  public final maxRangeMeters:Float;

  final mountPosition:Array<Float>;
  final mountRotation:Array<Float>;

  function new(sensorId:String, sensorFrameId:String, bodyLinkId:String,
      intrinsics:PinholeCameraIntrinsics, pointCloudPerception:PointCloudObstaclePerception,
      mountPosition:Array<Float>, mountRotation:Array<Float>,
      minRangeMeters:Float, maxRangeMeters:Float) {
    if (sensorId == null || sensorId.length == 0 || sensorFrameId == null ||
        sensorFrameId.length == 0 || bodyLinkId == null || bodyLinkId.length == 0 ||
        intrinsics == null || pointCloudPerception == null)
      throw "Depth camera perception requires authored camera identity, calibration, and point-cloud settings";
    if (mountPosition == null || mountPosition.length != 3 ||
        mountRotation == null || mountRotation.length != 4)
      throw "Depth camera mount requires a translation and xyzw quaternion";
    for (value in mountPosition) if (!Math.isFinite(value))
      throw "Depth camera mount translation must be finite";
    for (value in mountRotation) if (!Math.isFinite(value))
      throw "Depth camera mount rotation must be finite";
    var norm = 0.0;
    for (value in mountRotation) norm += value * value;
    if (Math.abs(norm - 1.0) > 0.000001)
      throw "Depth camera mount rotation must be a unit quaternion";
    if (!Math.isFinite(minRangeMeters) || minRangeMeters < 0.0 ||
        !Math.isFinite(maxRangeMeters) || maxRangeMeters <= minRangeMeters)
      throw "Depth camera range must be finite, non-negative, and ordered";
    this.sensorId = sensorId;
    this.sensorFrameId = sensorFrameId;
    this.bodyLinkId = bodyLinkId;
    this.intrinsics = intrinsics;
    this.pointCloudPerception = pointCloudPerception;
    this.minRangeMeters = minRangeMeters;
    this.maxRangeMeters = maxRangeMeters;
    this.mountPosition = mountPosition.copy();
    this.mountRotation = mountRotation.copy();
  }

  /** Builds the image projection and 3D mount transform from an authored camera. */
  public static function fromRobotModel(model:RobotModel, sensorId:String,
      bodyLinkId:String, bodyFrameId:String, outputFrameId:String,
      intrinsics:PinholeCameraIntrinsics, maxRangeMeters:Float,
      obstacleRadiusMeters:Float, ?minRangeMeters:Float = 0.05,
      ?minConfidence:Float = 0.85, ?clusterDistanceMeters:Float = 0.25,
      ?minHeightMeters:Float = -0.25, ?maxHeightMeters:Float = 2.5,
      ?minClusterPoints:Int = 3, ?maxPointCount:Int = 100000):DepthCameraObstaclePerception {
    if (model == null || sensorId == null || sensorId.length == 0)
      throw "Model-driven depth perception requires a robot model and camera sensor ID";
    var sensor:Null<robotkit.model.Sensor> = null;
    for (candidate in model.sensors) if (candidate != null && candidate.id == sensorId) {
      if (sensor != null) throw 'Camera sensor ID "$sensorId" is ambiguous';
      sensor = candidate;
    }
    if (sensor == null) throw 'Camera sensor "$sensorId" is missing from the robot model';
    if (sensor.kind != "camera")
      throw 'Sensor "$sensorId" must use the camera kind for depth images';
    var configuredSensor:robotkit.model.Sensor = cast sensor;
    if (bodyFrameId == null || bodyFrameId.length == 0 ||
        outputFrameId != bodyLinkId)
      throw "Depth obstacle output must use the authored body link frame";
    if (configuredSensor.frame != null && configuredSensor.frame.link.id != bodyLinkId)
      throw "Depth camera must be mounted on the body link";
    var sourceFrame = configuredSensor.frame == null ? bodyLinkId :
      configuredSensor.frame.id;
    var position = configuredSensor.frame == null ? [0.0, 0.0, 0.0] :
      configuredSensor.frame.position.copy();
    var rotation = configuredSensor.frame == null ? [0.0, 0.0, 0.0, 1.0] :
      configuredSensor.frame.rotation.copy();
    if (!Math.isFinite(maxRangeMeters) || maxRangeMeters <= 0.0 ||
        !Math.isFinite(minRangeMeters) || minRangeMeters < 0.0 ||
        minRangeMeters >= maxRangeMeters)
      throw "Depth camera range must be finite, non-negative, and ordered";
    var mountDistance = Math.pow(position[0] * position[0] +
      position[1] * position[1] + position[2] * position[2], 0.5);
    var bodyPointCloudMaxRange = maxRangeMeters + mountDistance + 1e-6;
    if (!Math.isFinite(bodyPointCloudMaxRange))
      throw "Depth camera maximum range and mount must have a finite combined range";
    var cloud = new PointCloudObstaclePerception(bodyPointCloudMaxRange,
      obstacleRadiusMeters, 0.0, minConfidence,
      clusterDistanceMeters, minHeightMeters, maxHeightMeters, minClusterPoints,
      maxPointCount, "point_cloud");
    return new DepthCameraObstaclePerception(sensorId, sourceFrame, bodyLinkId,
      intrinsics, cloud, position, rotation, minRangeMeters, maxRangeMeters);
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Perception requires sensor frames";
    var detections:Array<Detection> = [];
    var obstacles:Array<Obstacle> = [];
    for (frame in frames) {
      if (frame == null || frame.sensorId != sensorId || frame.kind != "camera" ||
          frame.frameId != sensorFrameId || frame.image == null) continue;
      var image = frame.image;
      if (image.encoding != "depth32f")
        throw 'Depth camera sensor "$sensorId" requires depth32f images';
      if (intrinsics.principalPointXPixels >= image.width ||
          intrinsics.principalPointYPixels >= image.height)
        throw 'Depth camera sensor "$sensorId" image is smaller than its principal point';
      var stride = sampleStride(image.width, image.height,
        pointCloudPerception.maxPointCount);
      var points:Array<Float> = [];
      for (v in 0...image.height) {
        if (v % stride != 0) continue;
        for (u in 0...image.width) {
          if (u % stride != 0) continue;
          var depth = image.depthAt(u, v);
          if (!Math.isFinite(depth) || depth <= 0.0) continue;
          // The rectified image uses optical axes: +x right, +y down, +z forward.
          var opticalX = (u - intrinsics.principalPointXPixels) * depth /
            intrinsics.focalLengthXPixels;
          var opticalY = (v - intrinsics.principalPointYPixels) * depth /
            intrinsics.focalLengthYPixels;
          var opticalRange = Math.pow(opticalX * opticalX + opticalY * opticalY +
            depth * depth, 0.5);
          if (opticalRange <= minRangeMeters || opticalRange >= maxRangeMeters) continue;
          var mounted = rotate(mountRotation, opticalX, opticalY, depth);
          points.push(mounted.x + mountPosition[0]);
          points.push(mounted.y + mountPosition[1]);
          points.push(mounted.z + mountPosition[2]);
        }
      }
      if (points.length == 0) continue;
      var bodyCloud = new SensorFrame(sensorId, "point_cloud", bodyLinkId,
        frame.sequence, frame.sourceTimestampNs, points,
        frame.receivedTimestampNs, bodyLinkId, [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0], frame.sourceClockId, frame.receivedClockId);
      var result = pointCloudPerception.observe([bodyCloud]);
      detections = detections.concat(result.detections());
      obstacles = obstacles.concat(result.obstacles());
    }
    return new PerceptionSnapshot(detections, obstacles);
  }

  static function sampleStride(width:Int, height:Int, maxPointCount:Int):Int {
    var pixels = width * height;
    var stride = 1;
    if (pixels > maxPointCount)
      stride = Std.int(Math.ceil(Math.pow(pixels / maxPointCount, 0.5)));
    while (sampledWidth(width, stride) * sampledWidth(height, stride) > maxPointCount)
      stride++;
    return stride;
  }

  static function sampledWidth(size:Int, stride:Int):Int
    return Std.int((size + stride - 1) / stride);

  static function rotate(rotation:Array<Float>, x:Float, y:Float,
      z:Float):{x:Float, y:Float, z:Float} {
    var qx = rotation[0];
    var qy = rotation[1];
    var qz = rotation[2];
    var qw = rotation[3];
    var tx = 2.0 * (qy * z - qz * y);
    var ty = 2.0 * (qz * x - qx * z);
    var tz = 2.0 * (qx * y - qy * x);
    return {
      x: x + qw * tx + qy * tz - qz * ty,
      y: y + qw * ty + qz * tx - qx * tz,
      z: z + qw * tz + qx * ty - qy * tx
    };
  }
}

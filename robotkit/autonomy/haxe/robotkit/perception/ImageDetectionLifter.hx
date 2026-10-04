package robotkit.perception;

import robotkit.streams.ImageDetectionObservation;

import haxe.Int64;
import robotkit.mobile.Pose2;
import robotkit.mobile.Pose3;
import robotkit.core.SensorFrame;
import visionkit.CameraModel;

/** Synchronous image-to-planar adapter. Depth is axial camera X in metres. */
class ImageDetectionLifter implements Perception {
  public final camera:CameraModel;
  public final cameraSensorId:String;
  public final depthSensorId:Null<String>;
  public final parentFrameId:Null<String>;
  public final parentFromCamera:Null<Pose3>;
  final observations:SensorFrame->ImageDetectionObservation;

  /** Supply depthSensorId for aligned depth, or a camera mount and parent frame for floor intersection. */
  public function new(camera:CameraModel, cameraSensorId:String,
      observations:SensorFrame->ImageDetectionObservation,
      ?depthSensorId:String, ?parentFrameId:String, ?parentFromCamera:Pose3) {
    if (camera == null || cameraSensorId == null || cameraSensorId.length == 0 || observations == null)
      throw "Image lifter requires a camera, sensor ID, and observation source";
    if ((depthSensorId == null || depthSensorId.length == 0) ==
        (parentFrameId == null || parentFrameId.length == 0 || parentFromCamera == null))
      throw "Image lifter requires exactly one depth or ground-plane mode";
    this.camera = camera; this.cameraSensorId = cameraSensorId;
    this.observations = observations; this.depthSensorId = depthSensorId;
    this.parentFrameId = parentFrameId; this.parentFromCamera = parentFromCamera;
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Image lifter requires sensor frames";
    var source:Null<SensorFrame> = null;
    var depth:Null<SensorFrame> = null;
    for (frame in frames) {
      if (frame.sensorId == cameraSensorId) {
        if (source != null) throw "Image lifter received duplicate camera frames";
        source = frame;
      }
      if (depthSensorId != null && frame.sensorId == depthSensorId) {
        if (depth != null) throw "Image lifter received duplicate depth frames";
        depth = frame;
      }
    }
    if (source == null || source.image == null || source.image.encoding != "rgb8" ||
        source.image.width != camera.width || source.image.height != camera.height)
      throw "Image lifter requires a matching rgb8 camera frame";
    var observation = observations(source);
    if (observation == null || observation.sensorId != source.sensorId ||
        observation.sourceFrameId != source.frameId ||
        Int64.compare(observation.sequence, source.sequence) != 0 ||
        Int64.compare(observation.sourceTimestampNs, source.sourceTimestampNs) != 0 ||
        Int64.compare(observation.receivedTimestampNs, source.receivedTimestampNs) != 0 ||
        observation.sourceClockId != source.sourceClockId ||
        observation.receivedClockId != source.receivedClockId)
      throw "Image detection observation does not match camera frame";
    if (depthSensorId != null) {
      if (depth == null || depth.image == null || depth.image.encoding != "depth32f" ||
          depth.image.width != camera.width || depth.image.height != camera.height ||
          depth.frameId != source.frameId ||
          Int64.compare(depth.sequence, source.sequence) != 0 ||
          Int64.compare(depth.sourceTimestampNs, source.sourceTimestampNs) != 0 ||
          Int64.compare(depth.receivedTimestampNs, source.receivedTimestampNs) != 0 ||
          depth.sourceClockId != source.sourceClockId ||
          depth.receivedClockId != source.receivedClockId)
        throw "Aligned depth frame does not match camera frame";
    }
    var result:Array<Detection> = [];
    var index = 0;
    for (box in observation.detections) {
      var pixel = {x: box.x + box.width * 0.5,
        y: depth == null ? box.y + box.height : box.y + box.height * 0.5};
      var ray = camera.unproject([pixel])[0];
      var point:{x:Float, y:Float, z:Float};
      var frameId:String;
      if (depth != null) {
        var values:Array<Float> = [];
        var left = Std.int(Math.max(0, Math.floor(box.x)));
        var top = Std.int(Math.max(0, Math.floor(box.y)));
        var right = Std.int(Math.min(camera.width, Math.ceil(box.x + box.width)));
        var bottom = Std.int(Math.min(camera.height, Math.ceil(box.y + box.height)));
        for (y in top...bottom) for (x in left...right) {
          var value = depth.image.depthAt(x, y);
          if (Math.isFinite(value) && value > 0) values.push(value);
        }
        if (values.length == 0) { index++; continue; }
        values.sort(function(a, b) return a < b ? -1 : a > b ? 1 : 0);
        var metres = values[Std.int(values.length / 2)];
        point = {x: metres, y: metres * ray.y / ray.x, z: metres * ray.z / ray.x};
        frameId = source.frameId;
      } else {
        var mount:Pose3 = cast parentFromCamera;
        var rotated = mount.rotate(ray.x, ray.y, ray.z);
        if (rotated.z >= -1e-9 || mount.z <= 0) { index++; continue; }
        var scale = -mount.z / rotated.z;
        point = {x: mount.x + scale * rotated.x,
          y: mount.y + scale * rotated.y, z: 0};
        frameId = parentFrameId;
      }
      result.push(new Detection(observation.pipelineId + "/" + Std.string(index),
        box.label, box.score, new Pose2(point.x, point.y, 0), frameId,
        observation.sequence, observation.sourceTimestampNs,
        observation.receivedTimestampNs, observation.sourceClockId,
        observation.receivedClockId));
      index++;
    }
    return new PerceptionSnapshot(result);
  }
}

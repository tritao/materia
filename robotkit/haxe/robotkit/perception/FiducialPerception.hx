package robotkit.perception;

import robotkit.localization.FrameTree2;
import robotkit.mobile.Pose2;
import robotkit.mobile.Pose3;
import robotkit.model.RobotModel;
import robotkit.world.SensorFrame;

/** Maps camera detector fiducials to configured pallet and dock targets. */
class FiducialPerception implements Perception {
  public final sensorId:String;
  public final sensorFrameId:Null<String>;
  public final minConfidence:Float;
  public final frameTree:Null<FrameTree2>;
  public final outputFrameId:Null<String>;
  public final bodyLinkId:Null<String>;

  final targetsByMarkerId:Map<Int,FiducialTargetConfig>;
  final cameraMountPose3:Null<Pose3>;

  public function new(sensorId:String, targets:Array<FiducialTargetConfig>,
      ?minConfidence:Float = 0.5, ?frameTree:FrameTree2,
      ?outputFrameId:String, ?sensorFrameId:String,
      ?cameraMountPose3:Pose3, ?bodyLinkId:String) {
    if (sensorId == null || sensorId.length == 0 || targets == null || targets.length == 0 ||
        !Math.isFinite(minConfidence) || minConfidence < 0.0 || minConfidence > 1.0 ||
        (sensorFrameId != null && sensorFrameId.length == 0))
      throw "Fiducial perception configuration is invalid";
    if (outputFrameId != null && (outputFrameId.length == 0 || frameTree == null))
      throw "Fiducial output frame requires a non-empty frame ID and a frame tree";
    if ((cameraMountPose3 == null) != (bodyLinkId == null) ||
        (bodyLinkId != null && bodyLinkId.length == 0))
      throw "3D fiducial transformation requires both a camera mount and body link ID";
    this.sensorId = sensorId;
    this.sensorFrameId = sensorFrameId;
    this.minConfidence = minConfidence;
    this.frameTree = frameTree;
    this.outputFrameId = outputFrameId;
    this.cameraMountPose3 = cameraMountPose3;
    this.bodyLinkId = bodyLinkId;
    targetsByMarkerId = new Map<Int,FiducialTargetConfig>();
    for (target in targets) {
      if (target == null) throw "Fiducial target configuration cannot be null";
      if (targetsByMarkerId.exists(target.markerId))
        throw 'Fiducial marker ID ${target.markerId} is configured more than once';
      targetsByMarkerId.set(target.markerId, target);
    }
  }

  /** Builds camera mount handling and source-frame checks from the robot model. */
  public static function fromRobotModel(model:RobotModel, sensorId:String,
      bodyLinkId:String, bodyFrameId:String, outputFrameId:String,
      targets:Array<FiducialTargetConfig>, ?minConfidence:Float = 0.5):FiducialPerception {
    if (model == null || sensorId == null || sensorId.length == 0)
      throw "Model-driven fiducial configuration requires a robot model and sensor ID";
    var sensor:Null<robotkit.model.Sensor> = null;
    for (candidate in model.sensors) if (candidate != null && candidate.id == sensorId) {
      if (sensor != null) throw 'Camera sensor ID "$sensorId" is ambiguous';
      sensor = candidate;
    }
    if (sensor == null) throw 'Camera sensor "$sensorId" is missing from the robot model';
    if (sensor.kind != "camera" && sensor.kind != "camera_detections")
      throw 'Sensor "$sensorId" must use the camera kind';
    var configuredSensor:robotkit.model.Sensor = cast sensor;
    if (outputFrameId != bodyLinkId && outputFrameId != bodyFrameId)
      throw "Model-driven 3D fiducials must output in the body frame";
    if (configuredSensor.frame != null && configuredSensor.frame.link.id != bodyLinkId)
      throw "Fiducial camera must be mounted on the body link";
    var sourceFrame = configuredSensor.frame == null ? bodyLinkId :
      configuredSensor.frame.id;
    var mountPose = configuredSensor.frame == null ? new Pose3() :
      new Pose3(configuredSensor.frame.position[0], configuredSensor.frame.position[1],
        configuredSensor.frame.position[2], configuredSensor.frame.rotation[0],
        configuredSensor.frame.rotation[1], configuredSensor.frame.rotation[2],
        configuredSensor.frame.rotation[3]);
    var tree:Null<FrameTree2> = null;
    if (configuredSensor.frame == null ||
        (Math.abs(configuredSensor.frame.rotation[0]) < 1e-6 &&
         Math.abs(configuredSensor.frame.rotation[1]) < 1e-6))
      tree = FrameTree2.fromRobotModel(model, bodyLinkId);
    return new FiducialPerception(sensorId, targets, minConfidence, tree,
      tree == null ? null : bodyLinkId, sourceFrame, mountPose, bodyLinkId);
  }

  /**
   * Reads legacy five-value detector records: marker ID, x, y, ENU yaw,
   * confidence. Unknown markers, low confidence, and non-finite poses are ignored.
   */
  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Perception requires sensor frames";
    var detections:Array<Detection> = [];
    var pallets:Array<Pallet> = [];
    var dockingTargets:Array<DockingTarget> = [];
    for (frame in frames) {
      if (frame == null || frame.sensorId != sensorId ||
          (frame.kind != "camera_detections" && frame.kind != "camera") ||
          (sensorFrameId != null && frame.frameId != sensorFrameId) ||
          frame.values.length == 0) continue;
      if (frame.values.length % 5 != 0)
        throw 'Camera sensor "$sensorId" must provide five-value fiducial records';
      var records = Std.int(frame.values.length / 5);
      for (index in 0...records) {
        var markerValue = frame.values.get(index * 5);
        if (!Math.isFinite(markerValue) || markerValue < 0.0 ||
            markerValue > 2147483647.0 || markerValue != Math.floor(markerValue))
          continue;
        var x = frame.values.get(index * 5 + 1);
        var y = frame.values.get(index * 5 + 2);
        var yaw = frame.values.get(index * 5 + 3);
        var confidence = frame.values.get(index * 5 + 4);
        if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(yaw) ||
            !Math.isFinite(confidence) || confidence < minConfidence ||
            confidence > 1.0 || !targetsByMarkerId.exists(Std.int(markerValue))) continue;
        appendObservation(frame, index, new FiducialMarkerObservation(
          Std.int(markerValue), new Pose2(x, y, yaw), confidence), detections,
          pallets, dockingTargets);
      }
    }
    return new PerceptionSnapshot(detections, [], pallets, dockingTargets);
  }

  /** Runs an injected detector on camera images and maps typed results to targets. */
  public function observeCameraFrames(frames:Array<SensorFrame>,
      detector:FiducialDetector):PerceptionSnapshot {
    if (frames == null || detector == null)
      throw "Camera perception requires sensor frames and a detector";
    var detections:Array<Detection> = [];
    var pallets:Array<Pallet> = [];
    var dockingTargets:Array<DockingTarget> = [];
    for (frame in frames) {
      if (frame == null || frame.sensorId != sensorId || frame.kind != "camera" ||
          frame.image == null ||
          (sensorFrameId != null && frame.frameId != sensorFrameId)) continue;
      var observations = detector.detect(frame);
      if (observations == null)
        throw 'Fiducial detector returned null for camera sensor "$sensorId"';
      for (index in 0...observations.length) {
        var observation = observations[index];
        if (observation == null) continue;
        appendObservation(frame, index, observation, detections, pallets,
          dockingTargets);
      }
    }
    return new PerceptionSnapshot(detections, [], pallets, dockingTargets);
  }

  function appendObservation(frame:SensorFrame, index:Int,
      observation:FiducialMarkerObservation, detections:Array<Detection>,
      pallets:Array<Pallet>, dockingTargets:Array<DockingTarget>):Void {
    if (observation == null || observation.confidence < minConfidence) return;
    var target = targetsByMarkerId.get(observation.markerId);
    if (target == null) return;
    var pose = observation.pose;
    var outputFrame = outputFrameId == null ? frame.frameId : cast outputFrameId;
    var sourceFrame = frame.frameId;
    var observationPose3 = observation.pose3;
    if (observationPose3 != null && cameraMountPose3 != null && bodyLinkId != null) {
      pose = cameraMountPose3.compose(observationPose3).planarPose();
      sourceFrame = bodyLinkId;
      outputFrame = bodyLinkId;
    }
    if (outputFrame != sourceFrame) {
      var transforms:FrameTree2 = cast frameTree;
      pose = transforms.lookup(outputFrame, sourceFrame).compose(pose);
    }
    var detection = new Detection(
      '${sensorId}:${Std.string(frame.sequence)}:$index', target.targetKind,
      observation.confidence, pose, outputFrame, frame.sequence,
      frame.sourceTimestampNs, frame.receivedTimestampNs, frame.sourceClockId,
      frame.receivedClockId);
    detections.push(detection);
    switch target.targetKind {
      case "pallet": pallets.push(new Pallet(detection, target.lengthMeters,
        target.widthMeters, target.heightMeters));
      case "dock":
        var approachOffset:Pose2 = cast target.approachOffset;
        dockingTargets.push(new DockingTarget(detection,
          pose.compose(approachOffset)));
      case _: // Constructor validation makes other kinds unreachable.
    }
  }
}

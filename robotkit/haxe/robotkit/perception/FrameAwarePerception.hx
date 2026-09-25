package robotkit.perception;

import robotkit.localization.FrameTree2;
import robotkit.localization.Localization;
import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.localization.RobotFrameTree2;
import robotkit.mobile.Pose2;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.world.RobotSnapshot;
import robotkit.world.SensorFrame;

/**
 * Transforms sensor-frame perception values into the localization frame.
 * `observe()` expects matching localization state; `observeRobotSnapshot()`
 * updates both localization and model frames from one robot observation.
 */
class FrameAwarePerception implements Perception {
  public final source:Perception;
  public final localization:Localization;
  public var frames(default, null):FrameTree2;

  public function new(source:Perception, localization:Localization,
      ?frames:FrameTree2 = null) {
    if (source == null || localization == null)
      throw "Frame-aware perception requires a source and localization";
    this.source = source;
    this.localization = localization;
    this.frames = frames == null ? new FrameTree2() : frames;
  }

  public function observe(sensorFrames:Array<SensorFrame>):PerceptionSnapshot {
    if (sensorFrames == null) throw "Perception requires sensor frames";
    var estimate:Null<LocalizationState> = localization.state();
    if (estimate == null || estimate.quality == LocalizationQuality.Invalid)
      throw "Frame-aware perception requires a valid localization state";

    var observed = source.observe(sensorFrames);
    var detections = [for (value in observed.detections()) transformDetection(value, estimate)];
    var obstacles = [for (value in observed.obstacles()) new Obstacle(
      transformDetection(value.detection, estimate), value.radiusMeters)];
    var pallets = [for (value in observed.pallets()) new Pallet(
      transformDetection(value.detection, estimate), value.lengthMeters,
      value.widthMeters, value.heightMeters)];
    var dockingTargets = [for (value in observed.dockingTargets()) {
      var detection = transformDetection(value.detection, estimate);
      new DockingTarget(detection,
        transformPose(value.approachPose, value.detection.frameId, estimate));
    }];
    return new PerceptionSnapshot(detections, obstacles, pallets, dockingTargets);
  }

  /**
   * Updates localization and model kinematics from one observation, then
   * transforms that observation's sensors into the resulting reference frame.
   */
  public function observeRobotSnapshot(snapshot:RobotSnapshot, model:RobotModel,
      blueprint:RobotRuntimeBlueprint, bodyLinkId:String):PerceptionSnapshot {
    if (snapshot == null) throw "Frame-aware perception requires a robot snapshot";
    var currentFrames = RobotFrameTree2.fromSnapshot(model, blueprint, snapshot,
      bodyLinkId);
    localization.update(snapshot);
    frames = currentFrames;
    return observe(snapshot.sensors.toArray());
  }

  function transformDetection(value:Detection,
      estimate:LocalizationState):Detection {
    return new Detection(value.id, value.kind, value.confidence,
      transformPose(value.pose, value.frameId, estimate), estimate.referenceFrame,
      value.sourceSequence, value.sourceTimestampNs, value.receivedTimestampNs,
      value.sourceClockId, value.receivedClockId);
  }

  function transformPose(pose:Pose2, sourceFrame:String,
      estimate:LocalizationState):Pose2 {
    if (sourceFrame == estimate.referenceFrame) return pose;
    var referenceFromSource = sourceFrame == estimate.bodyFrame
      ? estimate.pose
      : estimate.pose.compose(frames.lookup(estimate.bodyFrame, sourceFrame));
    return referenceFromSource.compose(pose);
  }
}

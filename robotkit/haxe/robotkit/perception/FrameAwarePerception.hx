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
  /** Reads the sensor frames before `source` does, with each one's sensor placed in the reference frame. */
  public final scanFilter:Null<ScanFilter>;
  /** Reports, with each observation, the free space the LiDAR scans show (in the reference frame). */
  public final freeSpace:Null<LidarFreeSpace>;

  public function new(source:Perception, localization:Localization,
      ?frames:FrameTree2 = null, ?scanFilter:ScanFilter, ?freeSpace:LidarFreeSpace) {
    if (source == null || localization == null)
      throw "Frame-aware perception requires a source and localization";
    this.source = source;
    this.localization = localization;
    this.frames = frames == null ? new FrameTree2() : frames;
    this.scanFilter = scanFilter;
    this.freeSpace = freeSpace;
  }

  public function observe(sensorFrames:Array<SensorFrame>):PerceptionSnapshot {
    if (sensorFrames == null) throw "Perception requires sensor frames";
    var estimate:Null<LocalizationState> = localization.state();
    if (estimate == null || estimate.quality == LocalizationQuality.Invalid)
      throw "Frame-aware perception requires a valid localization state";

    var filter = scanFilter;
    var scans = filter == null ? sensorFrames : [for (frame in sensorFrames)
      filter.applies(frame) ? filter.filter(frame, referenceFromFrame(frame.frameId, estimate)) : frame];
    var observed = source.observe(scans);
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
    var layout = freeSpace;
    var views:Array<FreeSpaceView> = layout == null ? [] : [for (frame in sensorFrames)
      if (frame.kind == "lidar" && frame.values.length > 0) layout.viewing(frame, referenceFromFrame(frame.frameId, estimate))];
    return new PerceptionSnapshot(detections, obstacles, pallets, dockingTargets,
      views.length == 0 ? null : new FreeSpaceViews(views));
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
    return referenceFromFrame(sourceFrame, estimate).compose(pose);
  }

  function referenceFromFrame(sourceFrame:String, estimate:LocalizationState):Pose2 {
    if (sourceFrame == estimate.referenceFrame) return new Pose2();
    return sourceFrame == estimate.bodyFrame
      ? estimate.pose
      : estimate.pose.compose(frames.lookup(estimate.bodyFrame, sourceFrame));
  }
}

/** Free where any of several views sees it free. */
private class FreeSpaceViews implements FreeSpaceView {
  final views:Array<FreeSpaceView>;

  public function new(views:Array<FreeSpaceView>) this.views = views;

  public function freeAt(x:Float, y:Float, radiusMeters:Float):Bool {
    for (view in views) if (view.freeAt(x, y, radiusMeters)) return true;
    return false;
  }
}

package robotkit.localization;

import robotkit.mobile.Pose2;
import robotkit.model.RobotModel;

/** Tree of static planar frame transforms with explicit target/source direction. */
class FrameTree2 {
  final transforms:Array<FrameTransform2> = [];

  public function new() {}

  /**
   * Builds planar transforms for model frames attached to one body link.
   * Frames on other links are omitted; resolving them requires link kinematics.
   * The model frame's z translation is ignored, but roll or pitch is rejected.
   */
  public static function fromRobotModel(model:RobotModel,
      bodyLinkId:String):FrameTree2 {
    if (model == null || bodyLinkId == null || bodyLinkId.length == 0)
      throw "Model frame compilation requires a robot model and body link ID";
    var bodyLinkCount = 0;
    for (link in model.links)
      if (link != null && link.id == bodyLinkId) bodyLinkCount++;
    if (bodyLinkCount != 1)
      throw 'Robot model has no unique body link "$bodyLinkId"';

    var result = new FrameTree2();
    for (frame in model.frames) {
      if (frame == null || frame.link == null || frame.link.id != bodyLinkId) continue;
      if (frame.position == null || frame.position.length != 3 ||
          frame.rotation == null || frame.rotation.length != 4)
        throw 'Model frame "${frame.id}" has an invalid planar mount';
      for (value in frame.position) if (!Math.isFinite(value))
        throw 'Model frame "${frame.id}" has a non-finite mount position';
      for (value in frame.rotation) if (!Math.isFinite(value))
        throw 'Model frame "${frame.id}" has a non-finite mount rotation';

      var qx = frame.rotation[0];
      var qy = frame.rotation[1];
      var qz = frame.rotation[2];
      var qw = frame.rotation[3];
      var norm = qx * qx + qy * qy + qz * qz + qw * qw;
      if (Math.abs(norm - 1.0) > 1e-6)
        throw 'Model frame "${frame.id}" mount rotation must be unit length';
      var verticalAlignment = 1.0 - 2.0 * (qx * qx + qy * qy);
      if (Math.abs(verticalAlignment - 1.0) > 1e-6)
        throw 'Model frame "${frame.id}" has roll or pitch and cannot enter a planar frame tree';
      var yaw = Math.atan2(2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
      result.add(new FrameTransform2(bodyLinkId, frame.id,
        new Pose2(frame.position[0], frame.position[1], yaw)));
    }
    return result;
  }

  /** Adds one parent-child edge; each child may have only one parent. */
  public function add(transform:FrameTransform2):Void {
    if (transform == null) throw "Frame tree cannot add a null transform";
    for (value in transforms) if (value.childFrame == transform.childFrame)
      throw 'Frame "${transform.childFrame}" already has a parent';
    transforms.push(transform);
    var path:Array<String> = [];
    var current = transform.childFrame;
    while (true) {
      if (path.indexOf(current) >= 0) {
        transforms.pop();
        throw "Frame transform would create a cycle";
      }
      path.push(current);
      var parent:Null<String> = null;
      for (value in transforms) if (value.childFrame == current) {
        parent = value.parentFrame;
        break;
      }
      if (parent == null) break;
      current = cast parent;
    }
  }

  /** Returns `target_T_source`: the source frame pose expressed in target coordinates. */
  public function lookup(targetFrame:String, sourceFrame:String):Pose2 {
    if (targetFrame == null || targetFrame.length == 0 ||
        sourceFrame == null || sourceFrame.length == 0)
      throw "Frame lookup requires non-empty frame IDs";
    if (targetFrame == sourceFrame) return new Pose2();
    var source = rootPose(sourceFrame);
    var target = rootPose(targetFrame);
    if (source.root != target.root)
      throw 'Frames "$targetFrame" and "$sourceFrame" are disconnected';
    var rootToTargetInverse = new Pose2().relativeTo(target.pose);
    return rootToTargetInverse.compose(source.pose);
  }

  /** Transforms a pose expressed in source coordinates into target coordinates. */
  public function transformPose(pose:Pose2, sourceFrame:String,
      targetFrame:String):Pose2 {
    if (pose == null) throw "Frame transform requires a pose";
    return lookup(targetFrame, sourceFrame).compose(pose);
  }

  public function count():Int return transforms.length;

  function rootPose(frame:String):FrameRootPose {
    var current = frame;
    var rootToFrame = new Pose2();
    var visited:Array<String> = [];
    while (true) {
      if (visited.indexOf(current) >= 0)
        throw "Frame tree contains a cycle";
      visited.push(current);
      var edge:Null<FrameTransform2> = null;
      for (value in transforms) if (value.childFrame == current) {
        edge = value;
        break;
      }
      if (edge == null) return new FrameRootPose(current, rootToFrame);
      var value:FrameTransform2 = cast edge;
      rootToFrame = value.childPoseInParent.compose(rootToFrame);
      current = value.parentFrame;
    }
  }
}

private class FrameRootPose {
  public final root:String;
  public final pose:Pose2;

  public function new(root:String, pose:Pose2) {
    this.root = root;
    this.pose = pose;
  }
}

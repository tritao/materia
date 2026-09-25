package robotkit.localization;

import robotkit.mobile.Pose2;

/** Tree of static planar frame transforms with explicit target/source direction. */
class FrameTree2 {
  final transforms:Array<FrameTransform2> = [];

  public function new() {}

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

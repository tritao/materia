package robotkit.spatial;

/**
 * Tree of static 3D frame transforms with explicit parent/child direction.
 * Mirrors `robotkit.localization.FrameTree2` for full rigid poses; unrelated
 * frame graphs (e.g. a BIM `project_T_map` registration edge alongside a
 * robot's own frame chain) may share one tree as long as each child has one
 * parent.
 */
class FrameTree3 {
  final transforms:Array<FrameTransform3> = [];

  public function new() {}

  /** Adds one parent-child edge; each child may have only one parent. */
  public function add(edge:FrameTransform3):Void {
    if (edge == null) throw "Frame tree cannot add a null transform";
    for (value in transforms) if (value.childFrame == edge.childFrame)
      throw 'Frame "${edge.childFrame}" already has a parent';
    transforms.push(edge);
    var path:Array<String> = [];
    var current = edge.childFrame;
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
  public function lookup(targetFrame:String, sourceFrame:String):Transform3 {
    if (targetFrame == null || targetFrame.length == 0 ||
        sourceFrame == null || sourceFrame.length == 0)
      throw "Frame lookup requires non-empty frame IDs";
    if (targetFrame == sourceFrame) return Transform3.identity();
    var source = rootTransform(sourceFrame);
    var target = rootTransform(targetFrame);
    if (source.root != target.root)
      throw 'Frames "$targetFrame" and "$sourceFrame" are disconnected';
    return target.transform.inverse().compose(source.transform);
  }

  public function count():Int return transforms.length;

  function rootTransform(frame:String):FrameRootTransform {
    var current = frame;
    var rootFromFrame = Transform3.identity();
    var visited:Array<String> = [];
    while (true) {
      if (visited.indexOf(current) >= 0)
        throw "Frame tree contains a cycle";
      visited.push(current);
      var edge:Null<FrameTransform3> = null;
      for (value in transforms) if (value.childFrame == current) {
        edge = value;
        break;
      }
      if (edge == null) return new FrameRootTransform(current, rootFromFrame);
      var value:FrameTransform3 = cast edge;
      rootFromFrame = value.parent_T_child.compose(rootFromFrame);
      current = value.parentFrame;
    }
  }
}

private class FrameRootTransform {
  public final root:String;
  public final transform:Transform3;

  public function new(root:String, transform:Transform3) {
    this.root = root;
    this.transform = transform;
  }
}

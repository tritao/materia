package robotkit.spatial;

/** Static 3D transform edge: `parent_T_child`, the child frame's pose in the parent frame. */
class FrameTransform3 {
  public final parentFrame:String;
  public final childFrame:String;
  public final parent_T_child:Transform3;

  public function new(parentFrame:String, childFrame:String, parent_T_child:Transform3) {
    if (parentFrame == null || parentFrame.length == 0 || childFrame == null ||
        childFrame.length == 0 || parentFrame == childFrame || parent_T_child == null)
      throw "Frame transform requires distinct frame IDs and a transform";
    this.parentFrame = parentFrame;
    this.childFrame = childFrame;
    this.parent_T_child = parent_T_child;
  }
}

package robotkit;

import RobotKitRuntime;

/** Native execution shape compiled from a Haxeon robot document. */
class RuntimeLayout {
  public final revision:Int;
  public final jointCount:Int;
  public final linkCount:Int;
  public final frameCount:Int;

  public function new(revision:Int, jointCount:Int, ?linkCount:Int = 0,
      ?frameCount:Int = 0) {
    if (revision < 0 || jointCount < 0 || jointCount > RobotKitRuntimeConstants.RK_MAX_JOINTS)
      throw "Invalid RobotKit runtime layout";
    this.revision = revision;
    this.jointCount = jointCount;
    this.linkCount = linkCount;
    this.frameCount = frameCount;
  }

  @:allow(Runtime)
  function nativeValue():rk_runtime_layout {
    var value = new rk_runtime_layout();
    value.set_struct_size(rk_runtime_layout.size());
    value.set_revision(haxe.Int64.ofInt(revision));
    value.set_joint_count(jointCount);
    value.set_link_count(linkCount);
    value.set_frame_count(frameCount);
    return value;
  }
}

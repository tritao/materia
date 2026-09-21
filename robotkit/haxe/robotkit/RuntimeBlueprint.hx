package robotkit;

import RobotKitRuntime;

/** Bulk native execution description emitted from the Haxeon robot model. */
class RuntimeBlueprint {
  public final revision:Int;
  public final jointCount:Int;
  public final linkCount:Int;
  public final frameCount:Int;
  public final joints:Array<RuntimeJointBlueprint> = [];

  public function new(revision:Int, jointCount:Int, linkCount:Int,
      ?frameCount:Int = 0) {
    if (revision < 0 || jointCount < 0 || jointCount > RobotKitRuntimeConstants.RK_MAX_JOINTS ||
        linkCount < 1)
      throw "Invalid RobotKit runtime blueprint";
    this.revision = revision;
    this.jointCount = jointCount;
    this.linkCount = linkCount;
    this.frameCount = frameCount;
  }

  public function addJoint(value:RuntimeJointBlueprint):Void {
    if (joints.length >= jointCount)
      throw "RobotKit runtime blueprint has too many joints";
    joints.push(value);
  }

  @:allow(Runtime)
  function nativeValue():rk_runtime_blueprint {
    if (joints.length != jointCount)
      throw "RobotKit runtime blueprint is missing joints";
    var value = new rk_runtime_blueprint();
    value.set_struct_size(rk_runtime_blueprint.size());
    value.set_revision(haxe.Int64.ofInt(revision));
    value.set_joint_count(jointCount);
    value.set_link_count(linkCount);
    value.set_frame_count(frameCount);
    for (index in 0...joints.length)
      value.set_joints(index, joints[index].nativeValue());
    return value;
  }
}

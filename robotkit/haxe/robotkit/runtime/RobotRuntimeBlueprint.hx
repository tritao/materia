package robotkit.runtime;

import RobotKitRuntime;

/**
 * Immutable-at-execution compiled robot description consumed by RobotRuntime.
 *
 * Keeping this boundary separate from the editable `robotkit.model.RobotModel`
 * model makes validation and native handle creation deterministic.
 */
class RobotRuntimeBlueprint {
  public final revision:Int;
  public final jointCount:Int;
  public final linkCount:Int;
  public final frameCount:Int;
  public final joints:Array<RobotRuntimeJointBlueprint> = [];

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

  public function addJoint(value:RobotRuntimeJointBlueprint):Void {
    if (joints.length >= jointCount)
      throw "RobotKit runtime blueprint has too many joints";
    joints.push(value);
  }

  @:allow(RobotRuntime, Simulation)
  function nativeValue():rk_robot_runtime_blueprint {
    if (joints.length != jointCount)
      throw "RobotKit runtime blueprint is missing joints";
    var value = new rk_robot_runtime_blueprint();
    value.set_struct_size(rk_robot_runtime_blueprint.size());
    value.set_revision(haxe.Int64.ofInt(revision));
    value.set_joint_count(jointCount);
    value.set_link_count(linkCount);
    value.set_frame_count(frameCount);
    for (index in 0...joints.length)
      value.set_joints(index, joints[index].nativeValue());
    return value;
  }

  /** Builds the low-level layout needed only by standalone native creation. */
  @:allow(RobotRuntime)
  function nativeLayout():rk_robot_runtime_layout {
    var value = new rk_robot_runtime_layout();
    value.set_struct_size(rk_robot_runtime_layout.size());
    value.set_revision(haxe.Int64.ofInt(revision));
    value.set_joint_count(jointCount);
    value.set_link_count(linkCount);
    value.set_frame_count(frameCount);
    return value;
  }
}

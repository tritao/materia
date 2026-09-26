package robotkit.runtime;

import RobotKitRuntime;

/** Compiled joint limits and default control rate for one runtime joint. */
class RobotRuntimeJointBlueprint {
  public final joint:Int;
  public final type:Int;
  public final parentLink:Int;
  public final childLink:Int;
  public final lowerLimit:Float;
  public final upperLimit:Float;
  public final maxEffort:Float;
  public final maxRate:Float;
  public final maxAcceleration:Float;
  public final parentFramePosition:Array<Float>;
  public final parentFrameRotation:Array<Float>;
  public final childFramePosition:Array<Float>;
  public final childFrameRotation:Array<Float>;
  public final axis:Array<Float>;

  public function new(joint:Int, type:Int, parentLink:Int, childLink:Int,
      lowerLimit:Float, upperLimit:Float, maxEffort:Float, ?maxRate:Float = 0.0,
      ?parentFramePosition:Array<Float>, ?parentFrameRotation:Array<Float>,
      ?childFramePosition:Array<Float>, ?childFrameRotation:Array<Float>, ?axis:Array<Float>,
      ?maxAcceleration:Float = 0.0) {
    this.joint = joint;
    this.type = type;
    this.parentLink = parentLink;
    this.childLink = childLink;
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.maxEffort = maxEffort;
    this.maxRate = maxRate;
    this.maxAcceleration = maxAcceleration;
    this.parentFramePosition = parentFramePosition == null ? [0.0, 0.0, 0.0] : parentFramePosition.copy();
    this.parentFrameRotation = parentFrameRotation == null ? [0.0, 0.0, 0.0, 1.0] : parentFrameRotation.copy();
    this.childFramePosition = childFramePosition == null ? [0.0, 0.0, 0.0] : childFramePosition.copy();
    this.childFrameRotation = childFrameRotation == null ? [0.0, 0.0, 0.0, 1.0] : childFrameRotation.copy();
    this.axis = axis == null ? [0.0, 0.0, 1.0] : axis.copy();
  }

  @:allow(RobotRuntimeBlueprint)
  function nativeValue():rk_robot_runtime_joint {
    var value = new rk_robot_runtime_joint();
    value.set_joint(joint);
    value.set_type(type);
    value.set_parent_link(parentLink);
    value.set_child_link(childLink);
    value.set_lower_limit(lowerLimit);
    value.set_upper_limit(upperLimit);
    value.set_max_effort(maxEffort);
    value.set_max_acceleration(maxAcceleration);
    for (i in 0...3) {
      value.set_parent_frame_position(i, parentFramePosition[i]);
      value.set_child_frame_position(i, childFramePosition[i]);
      value.set_axis(i, axis[i]);
    }
    for (i in 0...4) {
      value.set_parent_frame_rotation(i, parentFrameRotation[i]);
      value.set_child_frame_rotation(i, childFrameRotation[i]);
    }
    return value;
  }
}

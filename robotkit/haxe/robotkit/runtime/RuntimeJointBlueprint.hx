package robotkit.runtime;

import RobotKitRuntime;

class RuntimeJointBlueprint {
  public final joint:Int;
  public final type:Int;
  public final parentLink:Int;
  public final childLink:Int;
  public final lowerLimit:Float;
  public final upperLimit:Float;
  public final maxEffort:Float;

  public function new(joint:Int, type:Int, parentLink:Int, childLink:Int,
      lowerLimit:Float, upperLimit:Float, maxEffort:Float) {
    this.joint = joint;
    this.type = type;
    this.parentLink = parentLink;
    this.childLink = childLink;
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.maxEffort = maxEffort;
  }

  @:allow(RuntimeBlueprint)
  function nativeValue():rk_runtime_joint {
    var value = new rk_runtime_joint();
    value.set_joint(joint);
    value.set_type(type);
    value.set_parent_link(parentLink);
    value.set_child_link(childLink);
    value.set_lower_limit(lowerLimit);
    value.set_upper_limit(upperLimit);
    value.set_max_effort(maxEffort);
    return value;
  }
}

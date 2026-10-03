package robotkit.runtime;

import RobotKitRuntime;

/**
 * Compiled joint limits and default control rate for one runtime joint.
 * `maxRate` is the joint's speed limit; the runtime also enforces it on
 * buffered trajectory chunks.
 */
class RobotRuntimeJointBlueprint {
  public final joint:Int;
  public final type:Int;
  public final parentLink:Int;
  public final childLink:Int;
  public final lowerLimit:Float;
  public final upperLimit:Float;
  public final maxEffort:Null<Float>;
  public final maxRate:Null<Float>;
  public final maxAcceleration:Null<Float>;
  public final parentFramePosition:Array<Float>;
  public final parentFrameRotation:Array<Float>;
  public final childFramePosition:Array<Float>;
  public final childFrameRotation:Array<Float>;
  public final axis:Array<Float>;
  /** Passive dynamics: reflected inertia, viscous damping and dry friction. */
  /** How far past its limits the joint's end stops sit; see `JointLimits.overtravel`. */
  public var overtravel:Float = 0.0;
  /**
   * Servo gains of a motor joint that moves other joints through couplings (torque per unit of joint
   * position and of joint speed); zero stiffness is no servo. The joints coupled to it take no
   * commands of their own in a simulation.
   */
  public var servoStiffness:Float = 0.0;
  public var servoDamping:Float = 0.0;
  public var armature:Float = 0.0;
  public var damping:Float = 0.0;
  public var frictionLoss:Float = 0.0;
  public var limitTimeConstant:Float = 0.0;
  public var limitDampingRatio:Float = 0.0;
  public var limitImpedance:Array<Float> = [0.0, 0.0, 0.0, 0.0, 0.0];

  public function new(joint:Int, type:Int, parentLink:Int, childLink:Int,
      lowerLimit:Float, upperLimit:Float, maxEffort:Null<Float>, ?maxRate:Float,
      ?parentFramePosition:Array<Float>, ?parentFrameRotation:Array<Float>,
      ?childFramePosition:Array<Float>, ?childFrameRotation:Array<Float>, ?axis:Array<Float>,
      ?maxAcceleration:Float) {
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

  public function requireRate():Float {
    var value = maxRate;
    if (value == null) throw "Compiled joint has no speed limit";
    return value;
  }

  public function requireEffort():Float {
    var value = maxEffort;
    if (value == null) throw "Compiled joint has no effort limit";
    return value;
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
    value.set_max_effort(maxEffort == null ? 0.0 : maxEffort);
    value.set_max_acceleration(maxAcceleration == null ? 0.0 : maxAcceleration);
    value.set_max_velocity(maxRate == null ? 0.0 : maxRate);
    value.set_limit_flags((maxEffort == null ? 0 : RobotKitRuntimeConstants.RK_LIMIT_EFFORT) |
      (maxRate == null ? 0 : RobotKitRuntimeConstants.RK_LIMIT_VELOCITY) |
      (maxAcceleration == null ? 0 : RobotKitRuntimeConstants.RK_LIMIT_ACCELERATION));
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

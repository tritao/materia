package robotkit.runtime;

import RobotKitRuntime;

/** A compiled SI-unit relation between two runtime joint indices. */
class RobotRuntimeJointCouplingBlueprint {
  public final leader:Int;
  public final follower:Int;
  public final ratio:Float;
  public final offset:Float;
  /** Follower-coordinate stiffness; zero is an ideal coupling constraint. */
  public var stiffness:Float = 0.0;
  /** Simulation constitutive law in physical runtime joint indices. */
  public var stiffnessLaw:Null<Array<{compliance:Float,coordinates:Array<{joint:Int,coefficient:Float}>}>> = null;
  /** Damping of the coupling's response as a ratio of critical damping; zero is undamped. */
  public var dampingRatio:Float = 0.0;

  public function new(leader:Int, follower:Int, ratio:Float, offset:Float) {
    this.leader = leader;
    this.follower = follower;
    this.ratio = ratio;
    this.offset = offset;
  }

  public function nativeValue():rk_robot_joint_coupling {
    var value = new rk_robot_joint_coupling();
    value.set_leader(leader);
    value.set_follower(follower);
    value.set_ratio(ratio);
    value.set_offset(offset);
    return value;
  }
}

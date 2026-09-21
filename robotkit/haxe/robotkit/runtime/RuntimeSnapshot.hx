package robotkit.runtime;

import RobotKitRuntime;

/** Immutable copy of one native runtime state batch. */
class RuntimeSnapshot {
  public final sequence:Int;
  public final timestampNs:haxe.Int64;
  public final mode:Int;
  public final safety:Int;
  public final endpoint:Int;
  public final faultCode:Int;
  public final positions:Array<Float>;
  public final velocities:Array<Float>;
  public final efforts:Array<Float>;

  function new(value:rk_robot_snapshot) {
    sequence = haxe.Int64.toInt(value.get_sequence());
    timestampNs = value.get_timestamp_ns();
    mode = value.get_mode();
    safety = value.get_safety();
    endpoint = value.get_endpoint();
    faultCode = value.get_fault_code();
    var count = value.get_joint_count();
    positions = [];
    velocities = [];
    efforts = [];
    for (index in 0...count) {
      positions.push(value.get_position(index));
      velocities.push(value.get_velocity(index));
      efforts.push(value.get_effort(index));
    }
  }

  @:allow(Runtime)
  static function fromNative(value:rk_robot_snapshot):RuntimeSnapshot
    return new RuntimeSnapshot(value);
}

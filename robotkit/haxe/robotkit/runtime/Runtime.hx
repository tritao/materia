package robotkit.runtime;

import RobotKitRuntime;
import RobotKitSimKit;

/** Coarse-grained Haxeon façade over the native RobotKit runtime. */
class Runtime {
  final owner:Ownedrk_runtime;
  var disposed:Bool = false;

  @:allow(robotkit.runtime.Simulation)
  private function new(owner:Ownedrk_runtime) {
    this.owner = owner;
  }

  public static function create(layout:RuntimeLayout):Runtime {
    var result = RobotKitRuntime.rk_runtime_create(layout.nativeValue());
    check(result.status, "runtime.create");
    return new Runtime(result.out_runtime);
  }

  public static function createSim(blueprint:RuntimeBlueprint):Runtime {
    var result = RobotKitSimKit.rk_runtime_create_sim(blueprint.nativeValue());
    check(result.status, "runtime.createSim");
    return new Runtime(result.out_runtime);
  }

  public function start():Void {
    ensureLive();
    check(RobotKitRuntime.rk_runtime_start(owner.borrow()), "runtime.start");
  }

  public function stop():Void {
    if (disposed)
      return;
    check(RobotKitRuntime.rk_runtime_stop(owner.borrow()), "runtime.stop");
  }

  /** Submits all position targets in one native call. */
  public function submitPositions(positions:Array<Float>, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    ensureLive();
    if (positions.length > RobotKitRuntimeConstants.RK_MAX_JOINTS)
      throw "Too many RobotKit position targets";
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(haxe.Int64.ofInt(sequence));
    command.set_timestamp_ns(timestampNs == null ? haxe.Int64.ofInt(0) : timestampNs);
    command.set_kind(RobotKitRuntimeConstants.RK_COMMAND_JOINT_TARGETS);
    command.set_target_count(positions.length);
    for (index in 0...positions.length) {
      var target = new rk_joint_target();
      target.set_joint(index);
      target.set_mode(RobotKitRuntimeConstants.RK_TARGET_POSITION);
      target.set_target(positions[index]);
      target.set_max_rate(0.0);
      target.set_max_effort(0.0);
      command.set_targets(index, target);
    }
    check(RobotKitRuntime.rk_runtime_submit(owner.borrow(), command),
      "runtime.submitPositions");
  }

  /** Submits a stop to the owner thread; it never steps synchronously. */
  public function submitStop(sequence:Int, emergency:Bool):Void {
    ensureLive();
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(haxe.Int64.ofInt(sequence));
    command.set_timestamp_ns(haxe.Int64.ofInt(0));
    command.set_kind(emergency
      ? RobotKitRuntimeConstants.RK_COMMAND_EMERGENCY_STOP
      : RobotKitRuntimeConstants.RK_COMMAND_STOP);
    command.set_target_count(0);
    check(RobotKitRuntime.rk_runtime_submit(owner.borrow(), command),
      "runtime.submitStop");
  }

  public function step(timestampNs:haxe.Int64):Void {
    ensureLive();
    check(RobotKitRuntime.rk_runtime_step(owner.borrow(), timestampNs), "runtime.step");
  }

  public function snapshot():RuntimeSnapshot {
    ensureLive();
    var value = new rk_robot_snapshot();
    value.set_struct_size(rk_robot_snapshot.size());
    check(RobotKitRuntime.rk_runtime_snapshot_full(owner.borrow(), value).status,
      "runtime.snapshot");
    return RuntimeSnapshot.fromNative(value);
  }

  public function dispose():Void {
    if (disposed)
      return;
    stop();
    owner.close();
    disposed = true;
  }

  function ensureLive():Void {
    if (disposed)
      throw "RobotKit runtime has been disposed";
  }

  static function check(status:Int, operation:String):Void {
    if (status != RobotKitRuntimeConstants.RK_OK)
      throw '$operation failed with RobotKit status $status';
  }
}

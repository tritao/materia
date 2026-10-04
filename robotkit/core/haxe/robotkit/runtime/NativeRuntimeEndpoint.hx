package robotkit.runtime;
import RobotKitRuntime;
import haxe.Int64;
import runtime.memory.NativeSpan;
/** Owns the native endpoint handle. In-memory, serial and simulation factories share its ABI. */
class NativeRuntimeEndpoint implements RuntimeEndpoint {
  final owner:Ownedrk_robot_runtime;
  final captureContacts:Null<Void -> Array<RobotContact>>;
  var closed:Bool = false;
  public function new(owner:Ownedrk_robot_runtime, ?captureContacts:Void -> Array<RobotContact>) {
    this.owner = owner;
    this.captureContacts = captureContacts;
  }
  /** Used by simulation's native adapter to identify its attached robot. */
  public function nativeHandle():rk_robot_runtime {
    if (closed) throw "Runtime endpoint is closed";
    return owner.borrow();
  }
  /** Read a copied input observation; no fabricated edge is returned without device state. */
  public function deviceInput(switchId:String):Null<DeviceInputReading> {
    if (switchId == null || switchId.length == 0) throw "Device input requires a switch ID";
    var result = RobotKitRuntime.rk_robot_runtime_device_input(nativeHandle(), switchId);
    if (result.status == RobotKitRuntimeConstants.RK_ERROR_STALE_STATE) return null;
    RobotRuntime.check(result.status, "deviceInput");
    var input = result.out_observation;
    return new DeviceInputReading(input.get_active() != 0, input.get_source_timestamp_ns(),
      input.get_closing_count(), input.get_opening_count(), input.get_captured_steps(),
      input.get_captured_timestamp_ns(), input.get_captured_position());
  }
  public function deviceHomingControl(action:Int, sequence:Int64, scope:Int64,
      first:Int, second:Int, skewBound:Float):Int {
    if (action < 0 || action > 3 || first < 0 || first >= 64 || second < 0 || second >= 64)
      throw "Invalid device homing control";
    var control = new rk_device_homing_control();
    control.set_struct_size(rk_device_homing_control.size());
    control.set_action(action); control.set_sequence(sequence); control.set_scope(scope);
    control.set_first(first); control.set_second(second); control.set_skew_bound(skewBound);
    return RobotKitRuntime.rk_robot_runtime_device_homing_control(nativeHandle(), control);
  }
  public function deviceHomingStatus(sequence:Int64):Int
    return RobotKitRuntime.rk_robot_runtime_device_homing_status(nativeHandle(), sequence);

  public function start():Int return RobotKitRuntime.rk_robot_runtime_start(nativeHandle());
  public function submit(command:rk_robot_command):Int
    return RobotKitRuntime.rk_robot_runtime_submit(nativeHandle(), command);
  public function observe(snapshot:rk_robot_snapshot):Int
    return RobotKitRuntime.rk_robot_runtime_snapshot_full(nativeHandle(), snapshot).status;
  public function observeEndpoint(snapshot:rk_robot_snapshot):Int
    return RobotKitRuntime.rk_robot_runtime_snapshot_endpoint(nativeHandle(), snapshot).status;
  public function calibrateHome(offsets:Array<Float>, referenceJoints:Array<Int>):Int
    return RobotKitRuntime.rk_robot_runtime_calibrate_home(nativeHandle(), offsets, referenceJoints);
  public function calibrateCoordinates(offsets:Array<Float>):Int
    return RobotKitRuntime.rk_robot_runtime_calibrate_coordinates(nativeHandle(), offsets);
  public function calibrateHomeDrives(joints:Array<Int>, sideZeros:Array<Float>):Int {
    if (joints == null || sideZeros == null || joints.length == 0 || joints.length != sideZeros.length)
      return RobotKitRuntimeConstants.RK_ERROR_INVALID_ARGUMENT;
    return RobotKitRuntime.rk_robot_runtime_calibrate_home_drives(nativeHandle(), joints, sideZeros);
  }
  public function stop():Int return closed ? RobotKitRuntimeConstants.RK_OK :
    RobotKitRuntime.rk_robot_runtime_stop(nativeHandle());
  public function capabilities(value:rk_robot_capabilities):Int
    return RobotKitRuntime.rk_robot_runtime_capabilities(nativeHandle(), value).status;
  public function limitInput(joint:Int, active:Bool):Int
    return RobotKitRuntime.rk_robot_runtime_limit_input(nativeHandle(), joint, active ? 1 : 0);
  public function requireReference(joint:Int, required:Bool):Int
    return RobotKitRuntime.rk_robot_runtime_require_reference(nativeHandle(), joint, required ? 1 : 0);
  public function submitPlanArrays(header:rk_plan_header, starts:Array<Int64>, durations:Array<Int64>,
      degrees:Array<Int>, coefficients:Array<Float>, joints:Array<Int>, events:Array<rk_timed_event>):Int
    return RobotKitRuntime.rk_robot_runtime_submit_plan(nativeHandle(), header, starts,
      durations, degrees, coefficients, joints, events);
  public function submitPlanSpan(header:rk_plan_header, starts:NativeSpan<Int64>, durations:NativeSpan<Int64>,
      degrees:NativeSpan<Int>, coefficients:NativeSpan<Float>, joints:NativeSpan<Int>, events:Array<rk_timed_event>):Int
    return RobotKitRuntime.rk_robot_runtime_submit_plan_span(nativeHandle(), header, starts,
      durations, degrees, coefficients, joints, events);
  public function pollEvents(batch:rk_event_record_batch):Int
    return RobotKitRuntime.rk_robot_runtime_poll_events(nativeHandle(), batch);
  public function channelValue(channel:String, value:rk_event_value):Int
    return RobotKitRuntime.rk_robot_runtime_get_channel_value(nativeHandle(), channel, value);
  public function contacts():Array<RobotContact> return captureContacts == null ? [] : captureContacts();
  public function close():Void {
    if (closed) return;
    owner.close();
    closed = true;
  }
}

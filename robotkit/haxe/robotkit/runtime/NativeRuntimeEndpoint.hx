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
  public function start():Int return RobotKitRuntime.rk_robot_runtime_start(nativeHandle());
  public function submit(command:rk_robot_command):Int
    return RobotKitRuntime.rk_robot_runtime_submit(nativeHandle(), command);
  public function observe(snapshot:rk_robot_snapshot):Int
    return RobotKitRuntime.rk_robot_runtime_snapshot_full(nativeHandle(), snapshot).status;
  public function stop():Int return closed ? RobotKitRuntimeConstants.RK_OK :
    RobotKitRuntime.rk_robot_runtime_stop(nativeHandle());
  public function capabilities(value:rk_robot_capabilities):Int
    return RobotKitRuntime.rk_robot_runtime_capabilities(nativeHandle(), value).status;
  public function submitSegments(command:rk_robot_command, tag:Int64, starts:Array<Int64>,
      durations:Array<Int64>, degrees:Array<Int>, coefficients:Array<Float>):Int
    return RobotKitRuntime.rk_robot_runtime_submit_segments(nativeHandle(), command, tag,
      starts, durations, degrees, coefficients);
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

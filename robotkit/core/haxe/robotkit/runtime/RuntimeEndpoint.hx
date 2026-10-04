package robotkit.runtime;
import RobotKitRuntime;
import haxe.Int64;
import runtime.memory.NativeSpan;
/** Endpoint contract shared by cyclic and scheduled transports. Status codes are RK_*.
    Arrays/spans are borrowed only for a submit call; observations are copied out. */
interface RuntimeEndpoint {
  public function start():Int;
  public function submit(command:rk_robot_command):Int;
  public function observe(snapshot:rk_robot_snapshot):Int;
  public function stop():Int;
  public function capabilities(value:rk_robot_capabilities):Int;
  public function submitPlanArrays(header:rk_plan_header, starts:Array<Int64>, durations:Array<Int64>,
    degrees:Array<Int>, coefficients:Array<Float>, joints:Array<Int>, events:Array<rk_timed_event>):Int;
  public function submitPlanSpan(header:rk_plan_header, starts:NativeSpan<Int64>, durations:NativeSpan<Int64>,
    degrees:NativeSpan<Int>, coefficients:NativeSpan<Float>, joints:NativeSpan<Int>, events:Array<rk_timed_event>):Int;
  public function pollEvents(batch:rk_event_record_batch):Int;
  public function channelValue(channel:String, value:rk_event_value):Int;
  public function contacts():Array<RobotContact>;
  public function close():Void;
}

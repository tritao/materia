package tests;

import RobotKitRuntime;
import robotkit.runtime.NativeRuntimeEndpoint;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeJointBlueprint;

/** A transport supplied by the caller must receive every execution operation. */
class RuntimeEndpointTests {
  public static function run():Int {
    var blueprint = new RobotRuntimeBlueprint(1, 1, 2);
    blueprint.addJoint(new RobotRuntimeJointBlueprint(0,
      RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -1.0, 1.0, 1.0));
    var result = RobotKitRuntime.rk_robot_runtime_create(blueprint.nativeValue());
    RobotRuntime.check(result.status, "endpointTest.create");
    var endpoint = new TrackingEndpoint(result.out_runtime);
    var runtime = RobotRuntime.create(blueprint, endpoint);
    runtime.start();
    runtime.submitPositions([0.25], 7);
    var observation = runtime.snapshot();
    runtime.stop();
    if (endpoint.starts != 1 || endpoint.submits != 1 || endpoint.observations != 1 ||
        endpoint.stops != 1 || observation.q.length != 1)
      throw "RobotRuntime did not use its supplied endpoint for start, submit, observe and stop";
    if (endpoint.sequence != 7 || runtime.contacts().length != 0)
      throw "Endpoint changed command identity or fabricated contacts";
    runtime.dispose();
    runtime.dispose();
    if (endpoint.closes != 1) throw "Runtime did not release its endpoint exactly once";
    var rejected = false;
    try runtime.start() catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Disposed runtime accepted a start";
    return 4;
  }
}

private class TrackingEndpoint extends NativeRuntimeEndpoint {
  public var starts:Int = 0;
  public var submits:Int = 0;
  public var observations:Int = 0;
  public var stops:Int = 0;
  public var closes:Int = 0;
  public var sequence:Int = 0;
  public function new(owner:Ownedrk_robot_runtime) super(owner);
  override public function start():Int { starts++; return super.start(); }
  override public function submit(command:rk_robot_command):Int {
    submits++;
    sequence = haxe.Int64.toInt(command.get_sequence());
    return super.submit(command);
  }
  override public function observe(snapshot:rk_robot_snapshot):Int {
    observations++;
    return super.observe(snapshot);
  }
  override public function stop():Int { stops++; return super.stop(); }
  override public function close():Void { closes++; super.close(); }
}

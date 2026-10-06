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
    var capabilities = runtime.capabilities("test/endpoint");
    var modes = capabilities.controlModes;
    modes.resize(0);
    if (!capabilities.accepts(robotkit.core.JointTargetMode.Servo) ||
        capabilities.jointCount != 1 || !capabilities.execution.plans ||
        capabilities.execution.maximumPolynomialDegree != 5 ||
        capabilities.execution.maximumJoints != 64 || !capabilities.timing.deadlines)
      throw "Endpoint capabilities lost modes or bounded execution contracts";
    var encoded = robotkit.protocol.CapabilityCodec.encode(capabilities, haxe.Int64.ofInt(42));
    var wire:robotkit.protocol.RobotCapabilities = haxeon.wire.MessagePack.decode(
      haxeon.wire.MessagePack.encode(encoded));
    var decoded = robotkit.protocol.CapabilityCodec.decode(wire, "test/remote");
    if (decoded.execution.maximumSegments != 4096 || !decoded.execution.holdResume ||
        !decoded.execution.timedEvents || !decoded.execution.replacementBoundaries ||
        !decoded.accepts(robotkit.core.JointTargetMode.Servo))
      throw "Remote capability round trip lost limits or contracts";
    encoded.controlModes.push("unknown");
    var invalidModeRejected = false;
    try robotkit.protocol.CapabilityCodec.decode(encoded, "test/remote")
      catch (_:Dynamic) invalidModeRejected = true;
    if (!invalidModeRejected) throw "Unknown remote control mode was accepted";
    var bounded = new robotkit.runtime.RuntimeRobotAdapter("test/bounded", runtime, "bounded", [], ["joint"],
      false, false, "bounded fault", new robotkit.core.ExecutionCapabilities(true, 0, 1, 1,
        false, false, false, trajectorykit.validation.ValidationGuarantee.Unchecked));
    var tooLarge = new robotkit.execution.ExecutionPlanSubmission(haxe.Int64.ofInt(1),
      haxe.Int64.ofInt(1), haxe.Int64.ofInt(1), 0, [0.0], [0.0], [0.0],
      [new robotkit.execution.TrajectorySegment(haxe.Int64.ofInt(0), haxe.Int64.ofInt(1000000), [[0.0, 0.1]])]);
    var limitRejected = false;
    try bounded.submit(robotkit.core.RobotCommand.ExecutionPlan(tooLarge))
      catch (_:Dynamic) limitRejected = true;
    if (!limitRejected) throw "Adapter accepted a plan above its advertised polynomial degree";
    var inflatedRejected = false;
    try new robotkit.runtime.RuntimeRobotAdapter("test/inflated", runtime, "inflated", [], ["joint"],
      false, false, "inflated fault", new robotkit.core.ExecutionCapabilities(true, 6, 64, 4096,
        true, true, true, trajectorykit.validation.ValidationGuarantee.Unchecked))
      catch (_:Dynamic) inflatedRejected = true;
    if (!inflatedRejected) throw "Adapter allowed a policy to invent endpoint capabilities";
    bounded.close();
    runtime.dispose();
    runtime.dispose();
    if (endpoint.closes != 1) throw "Runtime did not release its endpoint exactly once";
    var rejected = false;
    try runtime.start() catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Disposed runtime accepted a start";
    return 9;
  }
}

private class TrackingEndpoint extends NativeRuntimeEndpoint {
  public var starts:Int = 0;
  public var submits:Int = 0;
  public var observations:Int = 0;
  public var stops:Int = 0;
  public var closes:Int = 0;
  public var sequence:Int = 0;
  public function new(owner:Ownedrk_robot_runtime)
    super(owner,new robotkit.time.SourceClock("robotkit.monotonic"));
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

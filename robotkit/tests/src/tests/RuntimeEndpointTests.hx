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
    var sensorPolls = 0;
    runtime.installSensorPoller(function(_) sensorPolls++);
    if (runtime.status() != robotkit.core.RobotStatus.Ready || sensorPolls != 0)
      throw "Status should read the endpoint without polling device sensors";
    endpoint.forceEndpointFault = true;
    if (runtime.status() != robotkit.core.RobotStatus.Fault)
      throw "Status lost endpoint faults";
    endpoint.forceEndpointFault = false;
    endpoint.forceSafetyFault = true;
    if (runtime.status() != robotkit.core.RobotStatus.Fault)
      throw "Status lost safety faults";
    endpoint.forceSafetyFault = false;
    if (runtime.status() != robotkit.core.RobotStatus.Ready)
      throw "Status retained a stale fault";
    var physical = runtime.physicalPositions();
    if (physical.length != observation.q.length || physical[0] != observation.q.get(0))
      throw "Physical position read changed endpoint coordinates";
    physical[0] = 12345.0;
    if (runtime.physicalPositions()[0] == 12345.0 || sensorPolls != 0)
      throw "Physical position reads must own their arrays and avoid device sensor polling";
    var joints = runtime.jointObservation();
    if (joints.positions.length != observation.q.length || joints.positions[0] != observation.q.get(0) ||
        joints.sourceTimestampNs != observation.sourceTimestampNs || sensorPolls != 0)
      throw "Joint observations must preserve counter coordinates and source time without polling sensors";
    joints.positions[0] = 12345.0;
    if (runtime.jointObservation().positions[0] == 12345.0)
      throw "Joint observations must own their coordinates";
    var authoredPosition = [1.0, 2.0, 3.0];
    var mount = new robotkit.core.SensorMount(authoredPosition, [0.0, 0.0, 0.0, 1.0]);
    var values = [0.25];
    var firstFrame = new robotkit.core.SensorFrame("sensor", "imu", "frame", haxe.Int64.ofInt(1),
      haxe.Int64.ofInt(10), values, null, "link", null, null, "source", "received", null, mount);
    var secondFrame = new robotkit.core.SensorFrame("sensor", "imu", "frame", haxe.Int64.ofInt(2),
      haxe.Int64.ofInt(20), [0.5], null, "link", null, null, "source", "received", null, mount);
    authoredPosition[0] = 999.0;
    values[0] = 999.0;
    var exposedMount = firstFrame.mountPosition.toArray();
    exposedMount[0] = 999.0;
    if (firstFrame.mountPosition != secondFrame.mountPosition || firstFrame.mountRotation != secondFrame.mountRotation)
      throw "Sensor frames should share immutable authored mount storage";
    if (firstFrame.mountPosition.get(0) != 1.0 || secondFrame.mountPosition.get(0) != 1.0 ||
        firstFrame.values.get(0) != 0.25 || secondFrame.values.get(0) != 0.5)
      throw "Shared mounts and per-frame values must remain isolated from mutable inputs and exports";
    if (firstFrame.sequence == secondFrame.sequence || firstFrame.sourceTimestampNs == secondFrame.sourceTimestampNs)
      throw "Sensor frames must retain independent observation identity";
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
    if (bounded.status() != robotkit.core.RobotStatus.Ready) throw "Adapter lost ready status";
    endpoint.forceSafetyFault = true;
    if (bounded.status() != robotkit.core.RobotStatus.Fault) throw "Adapter lost safety fault status";
    endpoint.forceSafetyFault = false;
    bounded.close();
    if (bounded.status() != robotkit.core.RobotStatus.Disconnected) throw "Closed adapter stayed connected";
    runtime.dispose();
    runtime.dispose();
    if (endpoint.closes != 1) throw "Runtime did not release its endpoint exactly once";
    var rejected = false;
    try runtime.start() catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Disposed runtime accepted a start";
    return 23;
  }
}

private class TrackingEndpoint extends NativeRuntimeEndpoint {
  public var forceEndpointFault:Bool = false;
  public var forceSafetyFault:Bool = false;
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
    var result = super.observe(snapshot);
    if (forceEndpointFault) snapshot.set_endpoint(RobotKitRuntimeConstants.RK_ENDPOINT_FAULT);
    if (forceSafetyFault) snapshot.set_safety(RobotKitRuntimeConstants.RK_SAFETY_FAULT);
    return result;
  }
  override public function stop():Int { stops++; return super.stop(); }
  override public function close():Void { closes++; super.close(); }
}

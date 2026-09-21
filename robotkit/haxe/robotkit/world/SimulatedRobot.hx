package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.runtime.RobotRuntime;

/**
 * Robot adapter over a runtime belonging to an externally owned
 * Simulation. Closing this adapter never stops or disposes that Simulation;
 * the embedding application controls the shared clock and shutdown order.
 */
class SimulatedRobot implements Robot {
  public final logicalId:RobotId;

  final runtime:RobotRuntime;
  final robotDescription:RobotDescription;
  final robotCapabilities:RobotCapabilities;
  var changeListener:Null < RobotId -> Void > = null;
  var commandSequence:Int = 0;
  var observedSequence:Int64 = Int64.ofInt(-1);
  var currentSensors:Array<SensorFrame> = [];
  var closed:Bool = false;

  public function new(id:RobotId, runtime:RobotRuntime, name:String,
      links:Array<String>, joints:Array<String>) {
    if (id == null || id.length == 0)
      throw "SimulatedRobot requires a non-empty logical ID";
    if (runtime == null)
      throw "SimulatedRobot requires a runtime";
    logicalId = id;
    this.runtime = runtime;
    robotDescription = new RobotDescription(id, name, links, joints);
    robotCapabilities = new RobotCapabilities(
      id,
      joints == null ? 0 : joints.length,
      true,
      false,
      false,
      false
    );
  }

  public function id():RobotId return logicalId;

  public function status():RobotStatus {
    if (closed)
      return Disconnected;
    var value = runtime.snapshot();
    return value.endpoint == RobotKitRuntimeConstants.RK_ENDPOINT_FAULT ||
      value.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ? Fault : Ready;
  }

  public function description():RobotDescription return robotDescription;

  public function capabilities():RobotCapabilities return robotCapabilities;

  public function snapshot():robotkit.world.RobotSnapshot {
    ensureOpen();
    var value = runtime.snapshot();
    observe(value);
    return new robotkit.world.RobotSnapshot(
      logicalId,
      value.sequence,
      value.sourceTimestampNs,
      value.q.toArray(),
      value.dq.toArray(),
      value.effort.toArray(),
      value.mode,
      value.faultCode,
      value.receivedTimestampNs,
      currentSensors
    );
  }

  public function fault():Null < RobotFault > {
    if (closed)
      return null;
    var value = runtime.snapshot();
    if (value.faultCode == 0 && value.safety != RobotKitRuntimeConstants.RK_SAFETY_FAULT)
      return null;
    return new RobotFault(logicalId, value.faultCode, "simulated runtime fault", true);
  }

  public function submit(command:RobotCommand):Void {
    ensureOpen();
    switch command {
      case JointPosition(joint, target, _):
        commandSequence++;
        runtime.submitPosition(joint, target, commandSequence);
    }
  }

  public function stop(mode:StopMode):Void {
    ensureOpen();
    commandSequence++;
    runtime.submitStop(commandSequence, mode == StopMode.Emergency);
  }

  public function sensors():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (frame in currentSensors)
      result.push(new SensorFrame(frame.sensorId, frame.kind, frame.frameId,
        frame.sequence, frame.sourceTimestampNs, frame.values.toArray(),
        frame.receivedTimestampNs));
    return result;
  }

  public function setChangeListener(listener:Null < RobotId -> Void >):Void {
    changeListener = listener;
  }

  /** The shared Simulation owns the runtime and remains externally managed. */
  public function close():Void {
    closed = true;
    changeListener = null;
  }

  function observe(value:robotkit.runtime.RobotSnapshot):Void {
    if (value.sequence == observedSequence)
      return;
    observedSequence = value.sequence;
    currentSensors = [
      new SensorFrame("imu", "imu", "base_link", value.sequence,
        value.sourceTimestampNs,
        [value.q.length > 0 ? value.q.get(0) : 0.0,
         value.dq.length > 0 ? value.dq.get(0) : 0.0,
         0.0, 0.0, 0.0, 9.81],
        value.receivedTimestampNs),
      new SensorFrame("lidar", "lidar", "base_link", value.sequence,
        value.sourceTimestampNs,
        [for (_ in 0...8) 10.0], value.receivedTimestampNs)
    ];
    var listener = changeListener;
    if (listener != null)
      listener(logicalId);
  }

  function ensureOpen():Void {
    if (closed)
      throw "SimulatedRobot has been closed";
  }
}

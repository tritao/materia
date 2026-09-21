package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeSnapshot;

/**
 * RobotInstance adapter over a runtime belonging to an externally owned
 * Simulation. Closing this adapter never stops or disposes that Simulation;
 * the embedding application controls the shared clock and shutdown order.
 */
class SimulatedRobot implements RobotInstance {
  public final logicalId:RobotId;

  final runtime:RobotRuntime;
  final robotDescription:RobotDescription;
  final robotCapabilities:RobotCapabilities;
  var changeListener:Null < Void -> Void > = null;
  var commandSequence:Int = 0;
  var observedSequence:Int = -1;
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

  public function snapshot():RobotSnapshot {
    ensureOpen();
    var value = runtime.snapshot();
    observe(value);
    return new RobotSnapshot(
      logicalId,
      Int64.ofInt(value.sequence),
      value.timestampNs,
      value.positions,
      value.velocities,
      value.efforts,
      value.mode,
      value.faultCode
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

  public function setChangeListener(listener:Null < Void -> Void >):Void {
    changeListener = listener;
  }

  /** The shared Simulation owns the runtime and remains externally managed. */
  public function close():Void {
    closed = true;
    changeListener = null;
  }

  function observe(value:RobotRuntimeSnapshot):Void {
    if (value.sequence == observedSequence)
      return;
    observedSequence = value.sequence;
    var listener = changeListener;
    if (listener != null)
      listener();
  }

  function ensureOpen():Void {
    if (closed)
      throw "SimulatedRobot has been closed";
  }
}

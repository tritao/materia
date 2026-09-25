package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.runtime.RobotRuntime;

/** Shared RobotWorld adapter for runtimes owned either by simulation or a device. */
class RuntimeRobotAdapter implements Robot {
  public final logicalId:RobotId;

  final runtime:RobotRuntime;
  final robotDescription:RobotDescription;
  final robotCapabilities:RobotCapabilities;
  final ownsRuntime:Bool;
  final faultMessage:String;
  var changeListener:Null < RobotId -> Void > = null;
  var commandSequence:Int = 0;
  var observedSequence:Int64 = Int64.ofInt(-1);
  var observedReceipt:Int64 = Int64.ofInt(-1);
  var currentSensors:Array<SensorFrame> = [];
  var closed:Bool = false;

  public function new(id:RobotId, runtime:RobotRuntime, name:String,
      links:Array<String>, joints:Array<String>, ?ownsRuntime:Bool = false,
      ?startRuntime:Bool = false, ?faultMessage:String = "robot runtime fault") {
    if (id == null || id.length == 0)
      throw "RuntimeRobotAdapter requires a non-empty logical ID";
    if (runtime == null)
      throw "RuntimeRobotAdapter requires a runtime";
    logicalId = id;
    this.runtime = runtime;
    this.ownsRuntime = ownsRuntime;
    this.faultMessage = faultMessage;
    robotDescription = new RobotDescription(id, name, links, joints);
    robotCapabilities = new RobotCapabilities(
      id, joints == null ? 0 : joints.length, true, true, true, false);
    if (startRuntime) {
      try runtime.start() catch (error:Dynamic) {
        if (ownsRuntime) runtime.dispose();
        throw error;
      }
    }
  }

  public function id():RobotId return logicalId;

  public function status():RobotStatus {
    if (closed) return Disconnected;
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
    return new RobotSnapshot(logicalId, value.sequence, value.sourceTimestampNs,
      value.q.toArray(), value.dq.toArray(), value.effort.toArray(), value.mode,
      value.faultCode, value.receivedTimestampNs, currentSensors);
  }

  public function fault():Null<RobotFault> {
    if (closed) return null;
    var value = runtime.snapshot();
    if (value.faultCode == 0 && value.safety != RobotKitRuntimeConstants.RK_SAFETY_FAULT)
      return null;
    return new RobotFault(logicalId, value.faultCode, faultMessage, true);
  }

  public function submit(command:RobotCommand):Void {
    ensureOpen();
    switch command {
      case JointTargets(targets, expiryNs):
        if (expiryNs != null && Int64.compare(expiryNs, Int64.ofInt(0)) != 0)
          throw "Runtime command deadlines are not supported; use bounded local intents";
        commandSequence++;
        runtime.submitTargets(targets, commandSequence);
    }
  }

  public function stop(mode:StopMode):Void {
    ensureOpen();
    commandSequence++;
    runtime.submitStop(commandSequence, mode == StopMode.Emergency);
  }

  public function sensors():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (frame in currentSensors) result.push(frame.copy());
    return result;
  }

  public function setChangeListener(listener:Null < RobotId -> Void >):Void {
    changeListener = listener;
  }

  public function close():Void {
    if (closed) return;
    closed = true;
    changeListener = null;
    if (ownsRuntime) runtime.dispose();
  }

  function observe(value:robotkit.runtime.RobotSnapshot):Void {
    if (value.sequence == observedSequence && value.receivedTimestampNs == observedReceipt)
      return;
    observedSequence = value.sequence;
    observedReceipt = value.receivedTimestampNs;
    currentSensors = RobotSensorFrames.fromRuntimeSnapshot(value);
    var listener = changeListener;
    if (listener != null) listener(logicalId);
  }

  function ensureOpen():Void {
    if (closed) throw "RuntimeRobotAdapter has been closed";
  }
}

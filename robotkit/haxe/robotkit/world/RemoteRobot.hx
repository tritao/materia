package robotkit.world;

import nativekit.ffi.NativeKit;
import NativeKitEvents;
import haxe.Int64;
import robotkit.client.RobotClient;
import robotkit.protocol.RobotStateMsg;
import robotkit.protocol.SensorFrameMsg;

/** Live robot adapter reached through RobotClient/RobotProtocol. */
class RemoteRobot implements Robot {
  public final logicalId:RobotId;

  final client:RobotClient;
  var currentSnapshot:RobotSnapshot;
  var currentFault:Null<RobotFault> = null;
  var currentSensors:Array<SensorFrame> = [];
  final sensorSourceReceipts:Map<String, Int64> = [];
  var changeListener:Null < RobotId -> Void > = null;

  public function new(id:RobotId) {
    if (id == null || id.length == 0) throw "RemoteRobot requires a non-empty logical ID";
    logicalId = id;
    client = new RobotClient("materia-world-" + id);
    currentSnapshot = new RobotSnapshot(id, Int64.ofInt(0), Int64.ofInt(0), [], [], [], 0, 0);
    client.stateListener = onState;
    client.faultListener = onFault;
    client.statusListener = onStatus;
    client.sensorListener = onSensor;
  }

  public function id():RobotId return logicalId;

  /** Negotiated wire identity, exposed for transport diagnostics only. */
  public function protocolRobotId():Null < Int64 > {
    var value = client.welcome;
    return value == null ? null : value.robotId;
  }

  /** Connects this adapter using the host's event pump. */
  public function connect(host:String, port:Int, events:NativeKitEvents):Void client.connectWithEvents(
    host,
    port,
    events
  );

  public function status():RobotStatus {
    if (currentFault != null && currentFault.fatal) return Fault;
    if (client.isReady()) return Ready;
    if (client.isConnected()) return Connecting;
    return Disconnected;
  }

  public function description():RobotDescription {
    var value = client.description;
    return value == null ? new RobotDescription(logicalId, logicalId, [], []) : new RobotDescription(
      logicalId,
      value.name,
      value.links,
      value.joints
    );
  }

  public function capabilities():RobotCapabilities {
    var value = client.capabilities;
    return value == null ? new RobotCapabilities(logicalId, 0, false, false, false, false) : new RobotCapabilities(
      logicalId,
      value.jointCount,
      value.supportsPosition,
      value.supportsVelocity,
      value.supportsEffort,
      value.supportsPrediction
    );
  }

  public function snapshot():RobotSnapshot return new RobotSnapshot(
    currentSnapshot.id,
    currentSnapshot.sourceSequence,
    currentSnapshot.sourceTimestampNs,
    currentSnapshot.positions.toArray(),
    currentSnapshot.velocities.toArray(),
    currentSnapshot.efforts.toArray(),
    currentSnapshot.mode,
    currentSnapshot.faultCode,
    currentSnapshot.receivedTimestampNs,
    currentSensors
  );

  public function sensors():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (frame in currentSensors)
      result.push(frame.copy());
    return result;
  }

  public function fault():Null < RobotFault > return currentFault;

  public function submit(command:RobotCommand):Void switch command {
    case JointPosition(joint, target, expiryNs):
      client.sendJointTarget(joint, 1, target, expiryNs);
  }

  public function stop(mode:StopMode):Void {
    client.stop("world stop", mode == StopMode.Emergency);
  }

  public function setChangeListener(listener:Null < RobotId -> Void >):Void {
    changeListener = listener;
  }

  public function close():Void client.close();

  function onState(value:RobotStateMsg):Void {
    currentSnapshot = new RobotSnapshot(
      logicalId,
      value.sequence,
      value.sourceTimestampNs,
      value.q,
      value.dq,
      value.effort,
      value.mode,
      value.fault,
      NativeKit.nk_time_now_ns()
    );
    notifyChanged();
  }

  function onFault(value:robotkit.protocol.Fault):Void {
    currentFault = new RobotFault(logicalId, value.code, value.message, value.fatal);
    notifyChanged();
  }

  function onSensor(value:SensorFrameMsg):Void {
    var welcome = client.welcome;
    if (welcome != null && Int64.compare(value.robotId, welcome.robotId) != 0)
      return;
    var frame = new SensorFrame(value.sensorId, value.kind, value.frameId,
      value.sequence, value.sourceTimestampNs, value.values,
      NativeKit.nk_time_now_ns(), value.linkId, value.mountPosition, value.mountRotation);
    var replaced = false;
    for (index in 0...currentSensors.length) {
      if (currentSensors[index].sensorId == frame.sensorId) {
        var old = currentSensors[index];
        if (old.sequence == frame.sequence && old.sourceTimestampNs == frame.sourceTimestampNs
            && sensorSourceReceipts.get(frame.sensorId) == value.receivedTimestampNs) return;
        currentSensors[index] = frame;
        replaced = true;
        break;
      }
    }
    if (!replaced) currentSensors.push(frame);
    sensorSourceReceipts.set(frame.sensorId, value.receivedTimestampNs);
    notifyChanged();
  }

  function onStatus():Void {
    if (client.lastFault == null) currentFault = null;
    notifyChanged();
  }

  function notifyChanged():Void {
    var listener = changeListener;
    if (listener != null) listener(logicalId);
  }
}

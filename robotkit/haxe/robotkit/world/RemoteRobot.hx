package robotkit.world;

import NativeKitEvents;
import haxe.Int64;
import robotkit.client.RobotClient;
import robotkit.protocol.RobotStateMsg;

/** RobotInstance adapter for a robot reached through RobotClient/RobotProtocol. */
class RemoteRobot implements RobotInstance {
  public final logicalId:RobotId;

  final client:RobotClient;
  var currentSnapshot:RobotSnapshot;
  var currentFault:Null<RobotFault> = null;
  var changeListener:Null < Void -> Void > = null;

  public function new(id:RobotId) {
    if (id == null || id.length == 0) throw "RemoteRobot requires a non-empty logical ID";
    logicalId = id;
    client = new RobotClient("materia-world-" + id);
    currentSnapshot = new RobotSnapshot(id, Int64.ofInt(0), Int64.ofInt(0), [], [], [], 0, 0);
    client.stateListener = onState;
    client.faultListener = onFault;
    client.statusListener = onStatus;
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

  public function snapshot():RobotSnapshot return currentSnapshot;

  public function fault():Null < RobotFault > return currentFault;

  public function submit(command:RobotCommand):Void switch command {
    case JointPosition(joint, target, expiryNs):
      client.sendJointTarget(joint, 1, target, expiryNs);
  }

  public function stop(mode:StopMode):Void {
    client.stop("world stop", mode == StopMode.Emergency);
  }

  public function setChangeListener(listener:Null < Void -> Void >):Void {
    changeListener = listener;
  }

  public function close():Void client.close();

  function onState(value:RobotStateMsg):Void {
    currentSnapshot = new RobotSnapshot(
      logicalId,
      value.sequence,
      value.timestampNs,
      value.q,
      value.dq,
      value.effort,
      value.mode,
      value.fault
    );
    notifyChanged();
  }

  function onFault(value:robotkit.protocol.Fault):Void {
    currentFault = new RobotFault(logicalId, value.code, value.message, value.fatal);
    notifyChanged();
  }

  function onStatus():Void {
    if (client.lastFault == null) currentFault = null;
    notifyChanged();
  }

  function notifyChanged():Void {
    var listener = changeListener;
    if (listener != null) listener();
  }
}

package robotkit.remote;

import robotkit.core.ExecutionCapabilities;
import robotkit.core.Robot;
import robotkit.core.RobotCapabilities;
import robotkit.core.RobotCommand;
import robotkit.core.RobotDescription;
import robotkit.core.RobotEvent;
import robotkit.core.RobotEventRing;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.RobotStatus;
import robotkit.core.SensorFrame;
import robotkit.core.StopMode;
import robotkit.core.TimingCapabilities;

import trajectorykit.validation.ValidationGuarantee;

import RobotKitRuntime;
import nativekit.ffi.NativeKit;
import NativeKitEvents;
import haxe.Int64;
import robotkit.client.RobotClient;
import robotkit.protocol.CameraFrameData;
import robotkit.protocol.RobotStateMsg;
import robotkit.protocol.SensorFrameMsg;

/** Live robot adapter reached through RobotClient/RobotProtocol. */
class RemoteRobot implements Robot {
  public final logicalId:RobotId;

  final client:RobotClient;
  var currentSnapshot:RobotSnapshot;
  var currentFault:Null<RobotFault> = null;
  var currentSensors:Array<SensorFrame> = [];
  final eventRing = new RobotEventRing();
  var lastWireObservationOrdinal:Int64 = Int64.ofInt(0);
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
    client.cameraListener = onCamera;
    client.subscribeCamera = false;
    client.imageDetectionListener = onImageDetection;
    client.subscribeObservations = false;
  }

  /** Request image frames before connecting. */
  public function enableCamera(?maxRateHz:Float = 0.0):Void {
    if (client.subscriptionLocked()) throw "Camera subscription must be set before connect";
    client.subscribeCamera = true;
    client.cameraMaxRateHz = maxRateHz;
  }

  /** Request robotd-produced observations before connecting. */
  public function enableObservations():Void {
    if (client.subscriptionLocked()) throw "Observation subscription must be set before connect";
    client.subscribeObservations = true;
  }

  public function streamCapabilities():Array<String> {
    var value = client.welcome;
    return value == null ? [] : value.capabilities.copy();
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
    return value == null ? new RobotCapabilities(logicalId, 0, [],
      ExecutionCapabilities.unavailable(), new TimingCapabilities(false, false, Unchecked))
      : robotkit.protocol.CapabilityCodec.decode(value, logicalId);
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
    currentSensors,
    currentSnapshot.sourceClockId,
    currentSnapshot.receivedClockId,
    currentSnapshot.safety,
    currentSnapshot.trajectoryQueueDepth, currentSnapshot.trajectoryActive,
    currentSnapshot.trajectoryTimeNs, currentSnapshot.trajectoryDurationNs,
    currentSnapshot.trajectoryTag, currentSnapshot.trajectoryTagTimeNs,
    currentSnapshot.sessionState, currentSnapshot.activePlanId,
    currentSnapshot.committedUntilNs, currentSnapshot.queueEndTimeNs
  );

  public function sensors():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (frame in currentSensors)
      result.push(frame.copy());
    return result;
  }

  public function fault():Null < RobotFault > return currentFault;

  public function events(afterOrdinal:Int64, max:Int):Array<RobotEvent>
    return eventRing.events(afterOrdinal, max);

  public function publishObservation(value:robotkit.perception.ImageDetectionObservation):RobotEvent {
    var event = eventRing.publish(value);
    notifyChanged();
    return event;
  }

  @:allow(tests.RobotEventTests)
  function onImageDetection(value:robotkit.protocol.ImageDetectionObservationMsg):Void {
    var welcome = client.welcome;
    if (welcome != null && Int64.compare(value.robotId, welcome.robotId) != 0) return;
    try {
      var observation = value.toObservation();
      if (Int64.compare(value.ordinal, lastWireObservationOrdinal) > 0) {
        var gap = Int64.sub(value.ordinal, lastWireObservationOrdinal);
        if (Int64.compare(gap, Int64.ofInt(1)) > 0) {
          var missing = Int64.sub(gap, Int64.ofInt(1));
          if (Int64.compare(missing, Int64.ofInt(2147483647)) > 0)
            throw "Observation gap exceeds supported range";
          eventRing.skip(Int64.toInt(missing));
        }
      }
      lastWireObservationOrdinal = value.ordinal;
      publishObservation(observation);
    } catch (error:Dynamic) {
      Sys.println('RemoteRobot ${logicalId}: discarded malformed image detection: $error');
    }
  }

  public function submit(command:RobotCommand):Void switch command {
    case JointTargets(targets, expiryNs):
      client.sendJointTargets(targets, expiryNs);
    case ExecutionPlan(plan): client.submitPlan(plan);
    case Hold: client.hold();
    case Resume: client.resume();
    case Abort: client.abort();
  }

  public function stop(mode:StopMode):Void {
    client.stop("world stop", mode == StopMode.Emergency);
  }

  public function resetSafety():Void client.resetSafety();

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
      NativeKit.nk_time_now_ns(),
      currentSensors,
      "unspecified",
      "robotkit.monotonic",
      value.safety,
      value.trajectoryQueueDepth, value.trajectoryActive,
      value.trajectoryTimeNs, value.trajectoryDurationNs,
      value.trajectoryTag, value.trajectoryTagTimeNs,
      value.sessionState, value.activePlanId,
      value.committedUntilNs, value.queueEndTimeNs
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
    updateSensor(frame, value.receivedTimestampNs);
  }

  @:allow(tests.RobotWorldTests)
  function onCamera(value:CameraFrameData):Void {
    var metadata = value.frame;
    var welcome = client.welcome;
    if (welcome != null && Int64.compare(metadata.robotId, welcome.robotId) != 0)
      return;
    var image = value.image();
    var frame = new SensorFrame(metadata.sensorId, metadata.kind,
      metadata.frameId, metadata.sequence, metadata.sourceTimestampNs, [],
      NativeKit.nk_time_now_ns(), metadata.linkId, metadata.mountPosition,
      metadata.mountRotation, metadata.sourceClockId, "robotkit.monotonic", image);
    updateSensor(frame, metadata.receivedTimestampNs);
  }

  function updateSensor(frame:SensorFrame, sourceReceipt:Int64):Void {
    var replaced = false;
    for (index in 0...currentSensors.length) {
      if (currentSensors[index].sensorId == frame.sensorId) {
        var old = currentSensors[index];
        if (old.sequence == frame.sequence && old.sourceTimestampNs == frame.sourceTimestampNs
            && sensorSourceReceipts.get(frame.sensorId) == sourceReceipt) return;
        currentSensors[index] = frame;
        replaced = true;
        break;
      }
    }
    if (!replaced) currentSensors.push(frame);
    sensorSourceReceipts.set(frame.sensorId, sourceReceipt);
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

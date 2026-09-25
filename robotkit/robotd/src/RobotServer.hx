package robotd;
import nativekit.ffi.NativeKitTypes;

import nativekit.ffi.NativeKit;
import NativeKitEventBytes;
import NativeKitEventValue;
import NativeKitEvents.NativeKitEventSubscription;
import NativeKitRuntime;
import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotSnapshot;
import robotkit.runtime.RobotSnapshotMailbox;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.behavior.RobotBehavior;
import robotkit.behavior.RobotBehaviorRunner;
import robotkit.protocol.Fault;
import robotkit.protocol.Hello;
import robotkit.protocol.JointTarget;
import robotkit.protocol.JointTargetValue;
import robotkit.protocol.JointTargets;
import robotkit.protocol.RobotCapabilities;
import robotkit.protocol.RobotDescription;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotFrame.RobotFrameStream;
import robotkit.protocol.RobotMessageType;
import robotkit.protocol.RobotProtocol;
import robotkit.protocol.RobotStateMsg;
import robotkit.protocol.SafetyReset;
import robotkit.protocol.Stop;
import robotkit.protocol.SensorFrameMsg;
import robotkit.transport.NativeTransport;
import robotkit.world.RobotSensorFrames;

/** Authoritative robot process boundary. RobotRuntime ownership stays off the network path. */
class RobotServer {
  final robot:RobotModel;
  final blueprint:RobotRuntimeBlueprint;
  final runtime:RobotRuntime;
  final simulation:Null<Simulation>;
  final nativeRuntime:NativeKitRuntime;
  final listener:OwnedListenerHandle;
  final port:Int;
  final robotId:Int;
  final subscription:NativeKitEventSubscription;
  final behaviorRunner:Null<RobotBehaviorRunner>;
  var client:Null<TransportHandle>;
  final observers:Array<TransportHandle> = [];
  final observerStreams:Map<Int, RobotFrameStream> = new Map<Int, RobotFrameStream>();
  final observerSessions:Map<Int, haxe.Int64> = new Map<Int, haxe.Int64>();
  final observerHello:Map<Int, Bool> = new Map<Int, Bool>();
  var stream:RobotFrameStream;
  final snapshots = new RobotSnapshotMailbox();
  var sessionId:haxe.Int64 = haxe.Int64.ofInt(0);
  var nextSessionId:haxe.Int64 = haxe.Int64.ofInt(1);
  var controllerGranted:Bool = true;
  var lastRequestSequence:haxe.Int64 = haxe.Int64.ofInt(0);
  var lastSentSnapshotSequence:haxe.Int64 = haxe.Int64.ofInt(-1);
  var helloComplete:Bool = false;
  var servedState:Bool = false;
  var onceMode:Bool = false;
  var behaviorPositions:Null<Array<Float>> = null;
  var behaviorSequence:Int = 0;
  var behaviorStopped:Bool = false;
  var disposed:Bool = false;

  public function new(robot:RobotModel, blueprint:RobotRuntimeBlueprint, runtime:RobotRuntime,
      simulation:Null<Simulation>, port:Int, robotId:Int, ?behavior:RobotBehavior) {
    this.robot = robot;
    this.blueprint = blueprint;
    this.runtime = runtime;
    this.simulation = simulation;
    this.port = port;
    this.robotId = robotId;
    behaviorRunner = behavior == null ? null : new RobotBehaviorRunner(behavior);
    nativeRuntime = NativeKitRuntime.start();
    try {
      if (simulation != null) simulation.start();
      else runtime.start();
      listener = NativeTransport.listen(port);
      stream = new RobotFrameStream();
      subscription = nativeRuntime.events.listen(onEvent);
    } catch (error:Dynamic) {
      try {
        if (simulation != null) simulation.stop();
        else runtime.stop();
      } catch (_:Dynamic) {}
      nativeRuntime.dispose();
      throw error;
    }
  }

  public function run(?once:Bool = false):Void {
    onceMode = once;
    Sys.println('robotd: listening on 127.0.0.1:$port');
    try {
      var stopped = false;
      while (!stopped) {
        publishSnapshot(false);
        var hadEvent = false;
        while (nativeRuntime.events.poll())
          hadEvent = true;
        if (!hadEvent)
          nativeRuntime.events.wait(0.01);
        if (onceMode && servedState && client == null && observers.length == 0)
          stopped = true;
      }
    } catch (error:Dynamic) {
      dispose();
      throw error;
    }
    dispose();
  }

  /** Runs one event-pump iteration; useful to tests and embedded hosts. */
  public function poll():Bool {
    publishSnapshot(false);
    var hadEvent = false;
    while (nativeRuntime.events.poll())
      hadEvent = true;
    return hadEvent;
  }

  public function dispose():Void {
    if (disposed)
      return;
    disposed = true;
    subscription.dispose();
    var currentClient = client;
    client = null;
    helloComplete = false;
    if (currentClient != null) {
      try { NativeTransport.close(currentClient); } catch (_:Dynamic) {}
    }
    for (observer in observers) {
      try { NativeTransport.close(observer); } catch (_:Dynamic) {}
    }
    observers.resize(0);
    observerStreams.clear();
    observerSessions.clear();
    observerHello.clear();
    listener.close();
    if (simulation != null) simulation.stop();
    else runtime.stop();
    nativeRuntime.dispose();
  }

  function onEvent(value:NativeKitEventValue):Void switch value {
    case Raw(kind, source, _, _, _, _, data):
      var currentClient = client;
      if (kind == EventKind.TransportAccepted &&
          source.rawValue() == listener.borrow().rawValue()) {
        accept(data);
      }
      else if (kind == EventKind.TransportData && currentClient != null &&
          source.rawValue() == currentClient.rawValue()) {
        receive(currentClient);
      }
      else if (kind == EventKind.TransportData) {
        var observer = findObserver(cast source);
        if (observer != null) receiveObserver(observer);
      }
      else if ((kind == EventKind.TransportClosed || kind == EventKind.TransportFailed) &&
          currentClient != null && source.rawValue() == currentClient.rawValue()) {
        closeClient(currentClient);
      }
      else if (kind == EventKind.TransportClosed || kind == EventKind.TransportFailed) {
        var observer = findObserver(cast source);
        if (observer != null) closeObserver(observer);
      }
    case _:
  }

  function accept(data:Bytes):Void {
    NativeKitEventBytes.requireMinimumSize(data, 32);
    var accepted = new TransportHandle(NativeKitEventBytes.readU32(data, 4));
    if (!accepted.isValid())
      return;
    if (client == null) {
      client = accepted;
      stream = new RobotFrameStream();
      servedState = false;
      helloComplete = false;
      controllerGranted = true;
      sessionId = nextSessionId;
      nextSessionId = haxe.Int64.add(nextSessionId, haxe.Int64.ofInt(1));
      lastRequestSequence = haxe.Int64.ofInt(0);
      lastSentSnapshotSequence = haxe.Int64.ofInt(-1);
    } else {
      observers.push(accepted);
      observerStreams.set(accepted.rawValue(), new RobotFrameStream());
      observerSessions.set(accepted.rawValue(), nextSessionId);
      observerHello.set(accepted.rawValue(), false);
      nextSessionId = haxe.Int64.add(nextSessionId, haxe.Int64.ofInt(1));
    }
  }

  function receive(transport:TransportHandle):Void {
    while (true) {
      var bytes:haxe.io.Bytes;
      try {
        bytes = NativeTransport.receive(transport);
      } catch (_:Dynamic) {
        closeClient(transport);
        return;
      }
      if (bytes.length == 0)
        return;
      for (frame in stream.push(bytes))
        handleFrame(frame);
    }
  }

  function receiveObserver(transport:TransportHandle):Void {
    var observerStream = observerStreams.get(transport.rawValue());
    if (observerStream == null) return;
    while (true) {
      var bytes:haxe.io.Bytes;
      try {
        bytes = NativeTransport.receive(transport);
      } catch (_:Dynamic) {
        closeObserver(transport);
        return;
      }
      if (bytes.length == 0) return;
      var frames = observerStream.push(bytes);
      for (frame in frames) handleObserverFrame(transport, frame);
    }
  }

  function handleFrame(frame:RobotFrame):Void switch frame.messageType {
    case RobotMessageType.Hello:
      handleHello(RobotProtocol.decodeHello(frame));
    case RobotMessageType.JointTarget:
      handleLegacyJointTarget(frame, RobotProtocol.decodeJointTarget(frame));
    case RobotMessageType.JointTargets:
      handleJointTargets(frame, RobotProtocol.decodeJointTargets(frame));
    case RobotMessageType.Stop:
      var value:Stop = RobotProtocol.decodeStop(frame);
      if (!controllerGranted || !sameRobot(value.robotId) || !validSession(frame)
          || !validCommandSequence(frame.sequence)) {
        sendFault(400, "invalid stop session", false);
      } else {
        try {
          runtime.submitStop(Int64.toInt(frame.sequence), value.emergency);
          lastRequestSequence = frame.sequence;
        } catch (error:Dynamic) {
          sendFault(422, 'stop rejected: $error', false);
        }
      }
    case RobotMessageType.SafetyReset:
      var value:SafetyReset = RobotProtocol.decodeSafetyReset(frame);
      if (!controllerGranted || !sameRobot(value.robotId) || !validSession(frame)
          || !validCommandSequence(frame.sequence)) {
        sendFault(400, "invalid safety reset session", false);
      } else {
        try {
          runtime.resetSafety(Int64.toInt(frame.sequence));
          lastRequestSequence = frame.sequence;
          servedState = true;
        } catch (error:Dynamic) {
          sendFault(422, 'safety reset rejected: $error', false);
        }
      }
    case _:
      sendFault(404, "unsupported RobotKit message", false);
  }

  function handleObserverFrame(transport:TransportHandle, frame:RobotFrame):Void {
    var observerSession = observerSessions.get(transport.rawValue());
    if (observerSession == null) return;
    if (frame.messageType != RobotMessageType.Hello) {
      sendTo(transport, observerSession,
        new RobotFrame(RobotMessageType.Fault,
          MessagePack.encode(new Fault(Int64.ofInt(robotId), 403,
            "observer is read-only", false)), 0, null,
          observerSession));
      return;
    }
    var value = RobotProtocol.decodeHello(frame);
    if (value.protocolVersion != 1) return;
    sendTo(transport, observerSession, RobotProtocol.welcome(
      new robotkit.protocol.Welcome(1, "robotd", observerSession,
        Int64.ofInt(robotId), false, Int64.ofInt(0)), observerSession));
    sendTo(transport, observerSession, RobotProtocol.description(new RobotDescription(
      Int64.ofInt(robotId), robot.name, [for (link in robot.links) link.name],
      [for (joint in robot.joints) joint.name]), observerSession));
    sendTo(transport, observerSession, RobotProtocol.capabilities(new RobotCapabilities(
      Int64.ofInt(robotId), blueprint.jointCount, true, true, true, false), observerSession));
    observerHello.set(transport.rawValue(), true);
    var latest = runtime.snapshot().withRobotId(Int64.ofInt(robotId));
    sendStateTo(transport, observerSession, latest);
  }

  function handleHello(value:Hello):Void {
    if (value.protocolVersion != 1) {
      sendFault(426, "unsupported RobotKit protocol version", true);
      return;
    }
    controllerGranted = value.requestedRole != "observer";
    send(RobotProtocol.welcome(new robotkit.protocol.Welcome(1, "robotd",
      sessionId, Int64.ofInt(robotId), controllerGranted,
      controllerGranted ? sessionId : Int64.ofInt(0)), sessionId));
    send(RobotProtocol.description(new RobotDescription(Int64.ofInt(robotId),
      robot.name, [for (link in robot.links) link.name],
      [for (joint in robot.joints) joint.name]), sessionId));
    send(RobotProtocol.capabilities(new RobotCapabilities(Int64.ofInt(robotId),
      blueprint.jointCount, true, true, true, false), sessionId));
    helloComplete = true;
    publishSnapshot(true);
    if (value.requestedRole == "observer")
      moveCurrentClientToObserver();
  }

  /**
   * The first accepted socket is provisionally the controller slot so the
   * server can read its Hello. If it asks for observation, move that same
   * session into the observer set and leave the controller slot available for
   * the next connection.
   */
  function moveCurrentClientToObserver():Void {
    var current = client;
    if (current == null) return;
    observers.push(current);
    observerStreams.set(current.rawValue(), stream);
    observerSessions.set(current.rawValue(), sessionId);
    observerHello.set(current.rawValue(), true);
    client = null;
    helloComplete = false;
    controllerGranted = true;
    sessionId = Int64.ofInt(0);
    lastRequestSequence = Int64.ofInt(0);
    lastSentSnapshotSequence = Int64.ofInt(-1);
  }

  function handleLegacyJointTarget(frame:RobotFrame, value:JointTarget):Void {
    handleJointTargets(frame, new JointTargets(value.robotId,
      [new JointTargetValue(value.joint, value.mode, value.target)],
      value.sequence, value.expiryNs));
  }

  function handleJointTargets(frame:RobotFrame, value:JointTargets):Void {
    if (!controllerGranted) {
      sendFault(403, "control lease not granted", false);
      return;
    }
    if (!validSession(frame) || !sameRobot(value.robotId)
        || Int64.compare(frame.sequence, value.sequence) != 0) {
      sendFault(400, "invalid RobotKit session or robot", false);
      return;
    }
    if (!validCommandSequence(value.sequence)) {
      sendFault(409, "stale RobotKit command sequence", false);
      return;
    }
    if (Int64.compare(value.expiryNs, Int64.ofInt(0)) != 0) {
      sendFault(422, "absolute deadlines require clock synchronization", false);
      return;
    }
    if (value.targets == null || value.targets.length == 0 ||
        value.targets.length > blueprint.jointCount) {
      sendFault(422, "joint target batch has an invalid size", false);
      return;
    }
    try {
      var targets:Array<robotkit.world.JointTarget> = [];
      var seen = new Map<Int, Bool>();
      for (target in value.targets) {
        if (target == null || target.joint < 0 || target.joint >= blueprint.jointCount
            || seen.exists(target.joint) || !Math.isFinite(target.target)) {
          sendFault(422, "joint target batch contains an invalid target", false);
          return;
        }
        seen.set(target.joint, true);
        var mode = switch target.mode {
          case 1: robotkit.world.JointTargetMode.Position;
          case 2: robotkit.world.JointTargetMode.Velocity;
          case 3: robotkit.world.JointTargetMode.Effort;
          case _: null;
        };
        if (mode == null) {
          sendFault(422, "joint target batch contains an unsupported mode", false);
          return;
        }
        targets.push(new robotkit.world.JointTarget(target.joint, mode, target.target));
      }
      runtime.submitTargets(targets, Int64.toInt(value.sequence));
      lastRequestSequence = value.sequence;
      servedState = true;
    } catch (error:Dynamic) {
      sendFault(422, 'command batch rejected: $error', false);
    }
  }

  function validSession(frame:RobotFrame):Bool
    return Int64.compare(frame.sessionId, sessionId) == 0;

  function validCommandSequence(sequence:Int64):Bool
    return Int64.compare(sequence, Int64.ofInt(0)) > 0
      && Int64.compare(sequence, lastRequestSequence) > 0;

  function sameRobot(value:haxe.Int64):Bool
    return Int64.compare(value, Int64.ofInt(robotId)) == 0;

  function publishSnapshot(forceSend:Bool):Void {
    var snapshot = runtime.snapshot().withRobotId(Int64.ofInt(robotId));
    snapshots.publish(snapshot);
    var latest = snapshots.latest();
    if (latest == null)
      return;
    applyBehavior(latest);
    if (!helloComplete || client == null)
      return;
    if (!forceSend && Int64.compare(latest.sequence, lastSentSnapshotSequence) <= 0)
      return;
    lastSentSnapshotSequence = latest.sequence;
    sendState(latest);
    for (observer in observers) {
      var key = observer.rawValue();
      var observerSession = observerSessions.get(key);
      if (observerHello.get(key) == true && observerSession != null)
        sendStateTo(observer, observerSession, latest);
    }
  }

  function applyBehavior(snapshot:RobotSnapshot):Void {
    var runner = behaviorRunner;
    if (runner == null)
      return;
    var intent = runner.update(snapshot, NativeKit.nk_time_now_ns());
    if (intent == null) {
      stopBehavior();
      return;
    }
    if (intent.joint < 0 || intent.joint >= blueprint.jointCount || intent.mode != 1) {
      stopBehavior();
      return;
    }
    var positions:Array<Float>;
    if (behaviorPositions == null) {
      positions = snapshot.q.toArray();
      behaviorPositions = positions;
    } else {
      positions = behaviorPositions;
    }
    for (index in 0...positions.length) {
      if (index != intent.joint && index < snapshot.q.length)
        positions[index] = snapshot.q.get(index);
    }
    positions[intent.joint] = intent.target;
    behaviorSequence++;
    runtime.submitPositions(positions, behaviorSequence, snapshot.sourceTimestampNs);
    behaviorStopped = false;
  }

  function stopBehavior():Void {
    if (behaviorStopped)
      return;
    behaviorSequence++;
    runtime.submitStop(behaviorSequence, false);
    behaviorStopped = true;
  }

  function sendState(snapshot:RobotSnapshot):Void {
    sendStateTo(client, sessionId, snapshot);
  }

  function sendStateTo(target:Null<TransportHandle>, targetSession:haxe.Int64,
      snapshot:RobotSnapshot):Void {
    if (target == null) return;
    var message = new RobotStateMsg(snapshot.robotId, snapshot.sequence,
      snapshot.sourceTimestampNs, snapshot.q.toArray(), snapshot.dq.toArray(), snapshot.effort.toArray(),
      snapshot.mode, snapshot.faultCode, snapshot.receivedTimestampNs, snapshot.safety);
    sendTo(target, targetSession, RobotProtocol.state(message, targetSession, snapshot.sequence,
      snapshot.sourceTimestampNs));
    for (sensor in RobotSensorFrames.fromRuntimeSnapshot(snapshot))
      sendTo(target, targetSession, RobotProtocol.sensorFrame(new SensorFrameMsg(
        snapshot.robotId, sensor.sensorId, sensor.kind, sensor.frameId, sensor.sequence,
        sensor.sourceTimestampNs, sensor.receivedTimestampNs, sensor.values.toArray(),
        sensor.linkId, sensor.mountPosition.toArray(), sensor.mountRotation.toArray()),
        targetSession, sensor.sequence, sensor.sourceTimestampNs));
  }

  function sendFault(code:Int, message:String, fatal:Bool):Void {
    var fault = new Fault(Int64.ofInt(robotId), code, message, fatal);
    send(new RobotFrame(RobotMessageType.Fault, MessagePack.encode(fault),
      0, null, sessionId));
  }

  function send(frame:RobotFrame):Void {
    var currentClient = client;
    if (currentClient == null)
      return;
    sendTo(currentClient, sessionId, frame);
  }

  function sendTo(target:Null<TransportHandle>, targetSession:haxe.Int64,
      frame:RobotFrame):Void {
    if (target == null) return;
    try {
      NativeTransport.send(target, frame.encode());
    } catch (_:Dynamic) {
      if (client != null && client.rawValue() == target.rawValue())
        closeClient(target);
      else closeObserver(target);
    }
  }

  function closeClient(currentClient:TransportHandle):Void {
    if (client != null && client.rawValue() == currentClient.rawValue()) {
      client = null;
      helloComplete = false;
    }
    try { NativeTransport.close(currentClient); } catch (_:Dynamic) {}
  }

  function findObserver(value:TransportHandle):Null<TransportHandle> {
    for (observer in observers)
      if (observer.rawValue() == value.rawValue()) return observer;
    return null;
  }

  function closeObserver(value:TransportHandle):Void {
    var index = -1;
    var candidateIndex = 0;
    for (candidate in observers) {
      if (candidate.rawValue() == value.rawValue()) { index = candidateIndex; break; }
      candidateIndex++;
    }
    if (index >= 0) observers.splice(index, 1);
    observerStreams.remove(value.rawValue());
    observerSessions.remove(value.rawValue());
    observerHello.remove(value.rawValue());
    try { NativeTransport.close(value); } catch (_:Dynamic) {}
  }
}

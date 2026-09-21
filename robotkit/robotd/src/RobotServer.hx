package robotd;

import NativeKit;
import NativeKit.EventKind;
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
import robotkit.protocol.RobotCapabilities;
import robotkit.protocol.RobotDescription;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotFrame.RobotFrameStream;
import robotkit.protocol.RobotMessageType;
import robotkit.protocol.RobotProtocol;
import robotkit.protocol.RobotStateMsg;
import robotkit.protocol.Stop;
import robotkit.transport.NativeTransport;

/** Authoritative robot process boundary. RobotRuntime ownership stays off the network path. */
class RobotServer {
  final robot:RobotModel;
  final blueprint:RobotRuntimeBlueprint;
  final runtime:RobotRuntime;
  final simulation:Simulation;
  final nativeRuntime:NativeKitRuntime;
  final listener:NativeKit.OwnedListenerHandle;
  final port:Int;
  final robotId:Int;
  final subscription:NativeKitEventSubscription;
  final behaviorRunner:Null<RobotBehaviorRunner>;
  var client:Null<NativeKit.TransportHandle>;
  var stream:RobotFrameStream;
  final snapshots = new RobotSnapshotMailbox();
  var sessionId:haxe.Int64 = haxe.Int64.ofInt(1);
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
      simulation:Simulation, port:Int, robotId:Int, ?behavior:RobotBehavior) {
    this.robot = robot;
    this.blueprint = blueprint;
    this.runtime = runtime;
    this.simulation = simulation;
    this.port = port;
    this.robotId = robotId;
    behaviorRunner = behavior == null ? null : new RobotBehaviorRunner(behavior);
    nativeRuntime = NativeKitRuntime.start();
    try {
      simulation.start();
      listener = NativeTransport.listen(port);
      stream = new RobotFrameStream();
      subscription = nativeRuntime.events.listen(onEvent);
    } catch (error:Dynamic) {
      try simulation.stop() catch (_:Dynamic) {}
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
        if (onceMode && servedState && client == null)
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
    if (currentClient != null)
      try NativeTransport.close(currentClient) catch (_:Dynamic) {}
    listener.close();
    simulation.stop();
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
      else if ((kind == EventKind.TransportClosed || kind == EventKind.TransportFailed) &&
          currentClient != null && source.rawValue() == currentClient.rawValue()) {
        closeClient(currentClient);
      }
    case _:
  }

  function accept(data:Bytes):Void {
    NativeKitEventBytes.requireMinimumSize(data, 32);
    var accepted = new NativeKit.TransportHandle(NativeKitEventBytes.readU32(data, 4));
    if (!accepted.isValid())
      return;
    if (client != null)
      closeClient(client);
    client = accepted;
    stream = new RobotFrameStream();
    servedState = false;
    helloComplete = false;
    lastSentSnapshotSequence = haxe.Int64.ofInt(-1);
  }

  function receive(transport:NativeKit.TransportHandle):Void {
    while (true) {
      var bytes = NativeTransport.receive(transport);
      if (bytes.length == 0)
        return;
      for (frame in stream.push(bytes))
        handleFrame(frame);
    }
  }

  function handleFrame(frame:RobotFrame):Void switch frame.messageType {
    case RobotMessageType.Hello:
      handleHello(RobotProtocol.decodeHello(frame));
    case RobotMessageType.JointTarget:
      handleJointTarget(frame, RobotProtocol.decodeJointTarget(frame));
    case RobotMessageType.Stop:
      var value:Stop = RobotProtocol.decodeStop(frame);
      if (!sameRobot(value.robotId) || !validSession(frame)
          || !validCommandSequence(frame.sequence)) {
        sendFault(400, "invalid stop session", false);
      } else {
        runtime.submitStop(Int64.toInt(frame.sequence), value.emergency);
        lastRequestSequence = frame.sequence;
      }
    case _:
      sendFault(404, "unsupported RobotKit message", false);
  }

  function handleHello(value:Hello):Void {
    if (value.protocolVersion != 1) {
      sendFault(426, "unsupported RobotKit protocol version", true);
      return;
    }
    send(RobotProtocol.welcome(new robotkit.protocol.Welcome(1, "robotd",
      sessionId, Int64.ofInt(robotId)), sessionId));
    send(RobotProtocol.description(new RobotDescription(Int64.ofInt(robotId),
      robot.name, [for (link in robot.links) link.name],
      [for (joint in robot.joints) joint.name]), sessionId));
    send(RobotProtocol.capabilities(new RobotCapabilities(Int64.ofInt(robotId),
      blueprint.jointCount, true, false, false, false), sessionId));
    helloComplete = true;
    publishSnapshot(true);
  }

  function handleJointTarget(frame:RobotFrame, value:JointTarget):Void {
    if (!validSession(frame) || !sameRobot(value.robotId)
        || Int64.compare(frame.sequence, value.sequence) != 0) {
      sendFault(400, "invalid RobotKit session or robot", false);
      return;
    }
    if (!validCommandSequence(value.sequence)) {
      sendFault(409, "stale RobotKit command sequence", false);
      return;
    }
    if (Int64.compare(value.expiryNs, Int64.ofInt(0)) != 0
        && Int64.compare(value.expiryNs, NativeKit.nk_time_now_ns()) < 0) {
      sendFault(408, "expired RobotKit command", false);
      return;
    }
    if (value.joint != 0 || value.mode != 1) {
      sendFault(422, "demo endpoint accepts position target joint 0 only", false);
      return;
    }
    lastRequestSequence = value.sequence;
    runtime.submitPositions([value.target], Int64.toInt(value.sequence));
    servedState = true;
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
      positions = snapshot.q.copy();
      behaviorPositions = positions;
    } else {
      positions = behaviorPositions;
    }
    for (index in 0...positions.length) {
      if (index != intent.joint && index < snapshot.q.length)
        positions[index] = snapshot.q[index];
    }
    positions[intent.joint] = intent.target;
    behaviorSequence++;
    runtime.submitPositions(positions, behaviorSequence, snapshot.timestampNs);
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
    var message = new RobotStateMsg(snapshot.robotId, snapshot.sequence,
      snapshot.timestampNs, snapshot.q.copy(), snapshot.dq.copy(), snapshot.effort.copy(),
      snapshot.mode, snapshot.faultCode);
    send(RobotProtocol.state(message, sessionId, snapshot.sequence,
      snapshot.timestampNs));
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
    try {
      NativeTransport.send(currentClient, frame.encode());
    } catch (_:Dynamic) {
      closeClient(currentClient);
    }
  }

  function closeClient(currentClient:NativeKit.TransportHandle):Void {
    if (client != null && client.rawValue() == currentClient.rawValue()) {
      client = null;
      helloComplete = false;
    }
    try NativeTransport.close(currentClient) catch (_:Dynamic) {}
  }
}

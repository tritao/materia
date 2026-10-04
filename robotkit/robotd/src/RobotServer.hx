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
import robotkit.runtime.SimulationHarness;
import robotkit.behavior.RobotBehavior;
import robotkit.behavior.RobotBehaviorRunner;
import robotkit.protocol.ControlHeartbeat;
import robotkit.protocol.Fault;
import robotkit.protocol.CameraFrame;
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
import robotkit.protocol.PixelFormat;
import robotkit.protocol.RobotStateMsg;
import robotkit.protocol.SafetyReset;
import robotkit.protocol.Stop;
import robotkit.protocol.PlanSubmission;
import robotkit.protocol.PathControl;
import robotkit.runtime.RobotRuntimeError;
import robotkit.protocol.SensorFrameMsg;
import robotkit.transport.NativeTransport;
import robotkit.core.RobotSensorFrames;
import robotkit.runtime.RuntimeRobotAdapter;
import robotkit.core.RobotEvent;
import robotkit.core.RobotEventRing;
import robotkit.perception.PerceptionHost;
import robotkit.perception.PerceptionPipelineRegistry;
import robotkit.deployment.PerceptionPipelineConfig;
import robotkit.protocol.ImageDetectionObservationMsg;

private enum ControlOwner {
  None;
  LocalBehavior;
  RemoteController(session:Int64);
}

/** Authoritative robot process boundary. RobotRuntime ownership stays off the network path. */
class RobotServer {
  public static inline final CONTROL_LEASE_TIMEOUT_MS:Int = 3000;

  final robot:RobotModel;
  final blueprint:RobotRuntimeBlueprint;
  final runtime:RobotRuntime;
  final simulation:Null<SimulationHarness>;
  final nativeRuntime:NativeKitRuntime;
  final listener:OwnedListenerHandle;
  final port:Int;
  final listenAddress:String;
  final robotId:Int;
  final subscription:NativeKitEventSubscription;
  final behaviorRunner:Null<RobotBehaviorRunner>;
  var client:Null<TransportHandle>;
  final observers:Array<TransportHandle> = [];
  final observerStreams:Map<Int, RobotFrameStream> = new Map<Int, RobotFrameStream>();
  final observerSessions:Map<Int, haxe.Int64> = new Map<Int, haxe.Int64>();
  final observerHello:Map<Int, Bool> = new Map<Int, Bool>();
  final outbound:Map<Int, OutboundScheduler> = new Map<Int, OutboundScheduler>();
  final bulkBudgetBytes:Int;
  final fixtureTick:Null<Void->Void>;
  final perception:Null<PerceptionHost>;
  final perceptionConsumers:Map<String, Array<String>> = new Map<String, Array<String>>();
  final perceptionSequences:Map<String, Int64> = new Map<String, Int64>();
  final producedEvents = new RobotEventRing();
  final hostedRobot:RuntimeRobotAdapter;
  var stream:RobotFrameStream;
  final snapshots = new RobotSnapshotMailbox();
  var sessionId:haxe.Int64 = haxe.Int64.ofInt(0);
  var nextSessionId:haxe.Int64 = haxe.Int64.ofInt(1);
  var controllerGranted:Bool = false;
  var lastRequestSequence:haxe.Int64 = haxe.Int64.ofInt(0);
  var lastSentSnapshotSequence:haxe.Int64 = haxe.Int64.ofInt(-1);
  var lastPublishedSnapshotSequence:haxe.Int64 = haxe.Int64.ofInt(-1);
  var helloComplete:Bool = false;
  var servedState:Bool = false;
  var onceMode:Bool = false;
  var behaviorPositions:Null<Array<Float>> = null;
  var controlOwner:ControlOwner = None;
  var runtimeSequence:Int64 = Int64.ofInt(0);
  var lastLeaseRenewalNs:Int64 = Int64.ofInt(0);
  var behaviorStopped:Bool = false;
  var disposed:Bool = false;

  public function new(robot:RobotModel, blueprint:RobotRuntimeBlueprint, runtime:RobotRuntime,
      simulation:Null<SimulationHarness>, port:Int, robotId:Int, ?behavior:RobotBehavior,
      ?listenAddress:String = "127.0.0.1", ?bulkBudgetBytes:Int = 0,
      ?fixtureTick:Void->Void, ?perceptionConfigs:Array<PerceptionPipelineConfig>) {
    this.robot = robot;
    this.blueprint = blueprint;
    this.runtime = runtime;
    hostedRobot = new RuntimeRobotAdapter("robotd", runtime, robot.name,
      [for (link in robot.links) link.name], [for (joint in robot.joints) joint.name]);
    this.simulation = simulation;
    this.port = port;
    this.listenAddress = listenAddress;
    this.robotId = robotId;
    this.bulkBudgetBytes = bulkBudgetBytes;
    this.fixtureTick = fixtureTick;
    var pipelines:Array<robotkit.perception.PerceptionPipeline> = [];
    if (perceptionConfigs != null) for (config in perceptionConfigs) if (config.host == "robotd") {
      pipelines.push(PerceptionPipelineRegistry.create(config, "robotd/" + config.id));
      perceptionConsumers.set(config.id, config.consumers.copy());
    }
    perception = pipelines.length == 0 ? null : new PerceptionHost(pipelines);
    behaviorRunner = behavior == null ? null : new RobotBehaviorRunner(behavior);
    if (behaviorRunner != null) controlOwner = LocalBehavior;
    nativeRuntime = NativeKitRuntime.start();
    try {
      if (simulation != null) simulation.start();
      else runtime.start();
      listener = NativeTransport.listen(port, listenAddress);
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
    Sys.println('robotd: listening on $listenAddress:$port');
    try {
      var stopped = false;
      while (!stopped) {
        if (fixtureTick != null) fixtureTick();
        publishSnapshot(false);
        pollPerception();
        var hadEvent = false;
        while (nativeRuntime.events.poll())
          hadEvent = true;
        checkControlLeaseTimeout();
        flushBulk();
        if (!hadEvent) {
          nativeRuntime.events.wait(0.01);
          checkControlLeaseTimeout();
        }
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
    if (fixtureTick != null) fixtureTick();
    publishSnapshot(false);
    pollPerception();
    var hadEvent = false;
    while (nativeRuntime.events.poll())
      hadEvent = true;
    checkControlLeaseTimeout();
    flushBulk();
    return hadEvent;
  }

  /** Current-session drop count for test and host diagnostics. */
  public function outboundDropCount(connectionId:Int, family:OutboundFamily):Int {
    var scheduler = outbound.get(connectionId);
    return scheduler == null ? 0 : scheduler.dropCount(family);
  }

  public function events(afterOrdinal:Int64, max:Int):Array<RobotEvent>
    return hostedRobot.events(afterOrdinal, max);

  function pollPerception():Void {
    if (perception == null) return;
    for (observation in perception.poll()) {
      var consumers = perceptionConsumers.get(observation.pipelineId);
      if (consumers == null) continue;
      var event = producedEvents.publish(observation);
      if (consumers.indexOf("local") >= 0) {
        var local = hostedRobot.publishObservation(observation);
        if (behaviorRunner != null) behaviorRunner.offerEvent(local);
      }
      if (consumers.indexOf("worldd") < 0) continue;
      var ordinal = RobotEventRing.ordinalOf(event);
      var message = ImageDetectionObservationMsg.fromObservation(Int64.ofInt(robotId), ordinal, observation);
      if (client != null && helloComplete) offerObservation(client, message);
      for (observer in observers.copy())
        if (observerHello.get(observer.rawValue()) == true) offerObservation(observer, message);
    }
  }

  function offerObservation(target:TransportHandle, value:ImageDetectionObservationMsg):Void {
    var scheduler = outbound.get(target.rawValue());
    if (scheduler == null || !scheduler.shouldOffer(OutboundFamily.Observation,
        value.producerId, value.ordinal, NativeKit.nk_time_now_ns())) return;
    var targetSession = target == client ? sessionId : observerSessions.get(target.rawValue());
    if (targetSession == null) return;
    scheduler.offer(OutboundFamily.Observation, value.producerId, value.ordinal,
      RobotProtocol.imageDetectionObservation(value, targetSession));
  }

  public function dispose():Void {
    if (disposed)
      return;
    hostedRobot.close();
    disposed = true;
    subscription.dispose();
    var currentClient = client;
    releaseRemoteOwner();
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
    outbound.clear();
    if (perception != null) perception.dispose();
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
    var scheduler:OutboundScheduler;
    try scheduler = new OutboundScheduler(accepted, bulkBudgetBytes)
    catch (error:Dynamic) {
      Sys.println('robotd: rejecting connection: $error');
      NativeTransport.close(accepted);
      return;
    }
    if (client == null) {
      client = accepted;
      outbound.set(accepted.rawValue(), scheduler);
      stream = new RobotFrameStream();
      servedState = false;
      helloComplete = false;
      controllerGranted = false;
      sessionId = nextSessionId;
      nextSessionId = haxe.Int64.add(nextSessionId, haxe.Int64.ofInt(1));
      lastRequestSequence = haxe.Int64.ofInt(0);
      lastSentSnapshotSequence = haxe.Int64.ofInt(-1);
    } else {
      observers.push(accepted);
      outbound.set(accepted.rawValue(), scheduler);
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

  function handleFrame(frame:RobotFrame):Void {
    switch frame.messageType {
    case RobotMessageType.Hello:
      try handleHello(RobotProtocol.decodeHello(frame))
      catch (error:Dynamic) sendFault(400, 'invalid Hello payload: $error', false);
    case RobotMessageType.JointTarget:
      handleLegacyJointTarget(frame, RobotProtocol.decodeJointTarget(frame));
    case RobotMessageType.JointTargets:
      handleJointTargets(frame, RobotProtocol.decodeJointTargets(frame));
    case RobotMessageType.PlanSubmission:
      try handlePlanSubmission(frame, RobotProtocol.decodePlanSubmission(frame))
      catch (error:Dynamic) sendFault(422, 'invalid plan payload: $error', false);
    case RobotMessageType.PathControl:
      try handlePathControl(frame, RobotProtocol.decodePathControl(frame))
      catch (error:Dynamic) sendFault(422, 'invalid path control payload: $error', false);
    case RobotMessageType.Stop:
      var value:Stop = RobotProtocol.decodeStop(frame);
      if (!remoteOwnsControl() || !sameRobot(value.robotId) || !validSession(frame)
          || !validCommandSequence(frame.sequence)) {
        sendFault(400, "invalid stop session", false);
      } else {
        try {
          runtime.submitStop64(nextRuntimeSequence(), value.emergency);
          lastRequestSequence = frame.sequence;
        } catch (error:Dynamic) {
          sendFault(422, 'stop rejected: $error', false);
        }
      }
    case RobotMessageType.SafetyReset:
      var value:SafetyReset = RobotProtocol.decodeSafetyReset(frame);
      if (!remoteOwnsControl() || !sameRobot(value.robotId) || !validSession(frame)
          || !validCommandSequence(frame.sequence)) {
        sendFault(400, "invalid safety reset session", false);
      } else {
        try {
          runtime.resetSafety64(nextRuntimeSequence());
          lastRequestSequence = frame.sequence;
          servedState = true;
        } catch (error:Dynamic) {
          sendFault(422, 'safety reset rejected: $error', false);
        }
      }
    case RobotMessageType.ControlHeartbeat:
      handleControlHeartbeat(frame, RobotProtocol.decodeControlHeartbeat(frame));
    case _:
      sendFault(404, "unsupported RobotKit message", false);
    }
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
    var value:Hello;
    try value = RobotProtocol.decodeHello(frame)
    catch (error:Dynamic) {
      sendTo(transport, observerSession, new RobotFrame(RobotMessageType.Fault,
        MessagePack.encode(new Fault(Int64.ofInt(robotId), 400,
          'invalid Hello payload: $error', false)), 0, null, observerSession));
      return;
    }
    if (value.protocolVersion != RobotFrame.VERSION) return;
    try {
      outbound.get(transport.rawValue()).configure(value.subscriptions);
    } catch (_:Dynamic) {
      sendTo(transport, observerSession, new RobotFrame(RobotMessageType.Fault,
        MessagePack.encode(new Fault(Int64.ofInt(robotId), 422,
          "invalid stream subscriptions", false)), 0, null, observerSession));
      return;
    }
    sendTo(transport, observerSession, RobotProtocol.welcome(
      new robotkit.protocol.Welcome(RobotFrame.VERSION, "robotd", observerSession,
        Int64.ofInt(robotId), false, Int64.ofInt(0), 0,
        OutboundPolicy.capabilities()), observerSession));
    sendTo(transport, observerSession, RobotProtocol.description(new RobotDescription(
      Int64.ofInt(robotId), robot.name, [for (link in robot.links) link.name],
      [for (joint in robot.joints) joint.name]), observerSession));
    sendTo(transport, observerSession, RobotProtocol.capabilities(robotkit.protocol.CapabilityCodec.encode(runtime.capabilities("robotd"), Int64.ofInt(robotId)), observerSession));
    observerHello.set(transport.rawValue(), true);
    var latest = runtime.snapshot().withRobotId(Int64.ofInt(robotId));
    sendStateTo(transport, observerSession, latest);
  }

  function handleHello(value:Hello):Void {
    if (helloComplete) {
      sendFault(409, "session already established", false);
      return;
    }
    if (value.protocolVersion != RobotFrame.VERSION) {
      sendFault(426, "unsupported RobotKit protocol version", true);
      return;
    }
    var scheduler = client == null ? null : outbound.get(client.rawValue());
    try {
      if (scheduler != null) scheduler.configure(value.subscriptions);
    } catch (_:Dynamic) {
      sendFault(422, "invalid stream subscriptions", false);
      return;
    }
    controllerGranted = value.requestedRole == "controller" &&
      switch controlOwner { case None: true; case _: false; };
    if (controllerGranted) {
      controlOwner = RemoteController(sessionId);
      lastLeaseRenewalNs = NativeKit.nk_time_now_ns();
    }
    send(RobotProtocol.welcome(new robotkit.protocol.Welcome(RobotFrame.VERSION, "robotd",
      sessionId, Int64.ofInt(robotId), controllerGranted,
      controllerGranted ? sessionId : Int64.ofInt(0),
      controllerGranted ? CONTROL_LEASE_TIMEOUT_MS : 0,
      OutboundPolicy.capabilities()), sessionId));
    send(RobotProtocol.description(new RobotDescription(Int64.ofInt(robotId),
      robot.name, [for (link in robot.links) link.name],
      [for (joint in robot.joints) joint.name]), sessionId));
    send(RobotProtocol.capabilities(robotkit.protocol.CapabilityCodec.encode(runtime.capabilities("robotd"), Int64.ofInt(robotId)), sessionId));
    helloComplete = true;
    publishSnapshot(true);
    if (!controllerGranted)
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
    controllerGranted = false;
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
    if (!controllerGranted || !remoteOwnsControl()) {
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
      var targets:Array<robotkit.core.JointTarget> = [];
      var seen = new Map<Int, Bool>();
      for (target in value.targets) {
        if (target == null || target.joint < 0 || target.joint >= blueprint.jointCount
            || seen.exists(target.joint) || !Math.isFinite(target.target)) {
          sendFault(422, "joint target batch contains an invalid target", false);
          return;
        }
        seen.set(target.joint, true);
        var mode = switch target.mode {
          case 1: robotkit.core.JointTargetMode.Position;
          case 2: robotkit.core.JointTargetMode.Velocity;
          case 3: robotkit.core.JointTargetMode.Effort;
          case _: null;
        };
        if (mode == null) {
          sendFault(422, "joint target batch contains an unsupported mode", false);
          return;
        }
        targets.push(new robotkit.core.JointTarget(target.joint, mode, target.target));
      }
      runtime.submitTargets64(targets, nextRuntimeSequence());
      lastRequestSequence = value.sequence;
      servedState = true;
    } catch (error:Dynamic) {
      sendFault(422, 'command batch rejected: $error', false);
    }
  }

  function handleControlHeartbeat(frame:RobotFrame, value:ControlHeartbeat):Void {
    if (!controllerGranted || !remoteOwnsControl() || !validSession(frame) ||
        !sameRobot(value.robotId) || Int64.compare(value.leaseId, sessionId) != 0) {
      sendFault(400, "invalid control lease heartbeat", false);
      return;
    }
    lastLeaseRenewalNs = NativeKit.nk_time_now_ns();
  }

  function handlePlanSubmission(frame:RobotFrame, value:PlanSubmission):Void {
    if (!controllerGranted || !remoteOwnsControl()) {
      sendFault(403, "control lease not granted", false);
      return;
    }
    if (!validSession(frame) || !sameRobot(value.robotId) ||
        !validCommandSequence(frame.sequence)) {
      sendFault(400, "invalid plan session, robot, or sequence", false);
      return;
    }
    if (!runtime.capabilities("robotd").execution.plans) {
      sendFault(422, "runtime does not support execution plans", false);
      return;
    }
    try {
      var plan = value.toWorld();
      if (plan.startPosition.length != blueprint.jointCount)
        throw "plan joint count does not match robot";
      runtime.submitPlan(plan, nextRuntimeSequenceInt());
      lastRequestSequence = frame.sequence;
      servedState = true;
    } catch (error:RobotRuntimeError) {
      sendFault(error.status, error.toString(), false);
    } catch (error:Dynamic) {
      sendFault(422, 'plan rejected: $error', false);
    }
  }

  function handlePathControl(frame:RobotFrame, value:PathControl):Void {
    if (!controllerGranted || !remoteOwnsControl()) {
      sendFault(403, "control lease not granted", false);
      return;
    }
    if (!validSession(frame) || !sameRobot(value.robotId) ||
        !validCommandSequence(frame.sequence)) {
      sendFault(400, "invalid path control session, robot, or sequence", false);
      return;
    }
    try {
      var sequence = nextRuntimeSequenceInt();
      switch value.action {
        case 1: runtime.submitHold(sequence);
        case 2: runtime.submitResume(sequence);
        case 3: runtime.submitAbort(sequence);
        case _: throw "invalid path control action";
      }
      lastRequestSequence = frame.sequence;
      servedState = true;
    } catch (error:RobotRuntimeError) {
      sendFault(error.status, error.toString(), false);
    } catch (error:Dynamic) {
      sendFault(422, 'path control rejected: $error', false);
    }
  }

  function nextRuntimeSequenceInt():Int {
    var sequence = nextRuntimeSequence();
    if (Int64.compare(sequence, Int64.ofInt(0x7fffffff)) > 0)
      throw "runtime command sequence exhausted";
    return Int64.toInt(sequence);
  }

  function checkControlLeaseTimeout():Void {
    if (!controllerGranted || !remoteOwnsControl() ||
        Int64.compare(lastLeaseRenewalNs, Int64.ofInt(0)) <= 0)
      return;
    var elapsedNs = Int64.sub(NativeKit.nk_time_now_ns(), lastLeaseRenewalNs);
    var timeoutNs = Int64.fromFloat(CONTROL_LEASE_TIMEOUT_MS * 1000000.0);
    if (Int64.compare(elapsedNs, timeoutNs) < 0)
      return;
    Sys.println('robotd: control lease expired for session ${sessionId}');
    var current = client;
    if (current != null)
      closeClient(current);
    else
      releaseRemoteOwner();
  }

  function validSession(frame:RobotFrame):Bool
    return Int64.compare(frame.sessionId, sessionId) == 0;

  function validCommandSequence(sequence:Int64):Bool
    return Int64.compare(sequence, Int64.ofInt(0)) > 0
      && Int64.compare(sequence, lastRequestSequence) > 0;

  function nextRuntimeSequence():Int64 {
    runtimeSequence = Int64.add(runtimeSequence, Int64.ofInt(1));
    return runtimeSequence;
  }

  function remoteOwnsControl():Bool return switch controlOwner {
    case RemoteController(ownerSession): Int64.compare(ownerSession, sessionId) == 0;
    case _: false;
  }

  function releaseRemoteOwner():Void {
    if (!remoteOwnsControl()) return;
    controlOwner = None;
    behaviorPositions = null;
    if (behaviorRunner != null) behaviorRunner.reset();
    runtime.submitStop64(nextRuntimeSequence(), true);
  }

  function sameRobot(value:haxe.Int64):Bool
    return Int64.compare(value, Int64.ofInt(robotId)) == 0;

  function publishSnapshot(forceSend:Bool):Void {
    var snapshot = runtime.snapshot().withRobotId(Int64.ofInt(robotId));
    snapshots.publish(snapshot);
    var latest = snapshots.latest();
    if (latest == null)
      return;
    if (perception != null) for (sensor in RobotSensorFrames.fromRuntimeSnapshot(latest)) {
      if (sensor.image == null) continue;
      var last = perceptionSequences.get(sensor.sensorId);
      if (last != null && Int64.compare(sensor.sequence, last) == 0) continue;
      perceptionSequences.set(sensor.sensorId, sensor.sequence);
      perception.submit(sensor);
    }
    applyBehavior(latest);
    if (!forceSend && Int64.compare(latest.sequence, lastPublishedSnapshotSequence) <= 0)
      return;
    lastPublishedSnapshotSequence = latest.sequence;
    if (helloComplete && client != null &&
        (forceSend || Int64.compare(latest.sequence, lastSentSnapshotSequence) > 0)) {
      lastSentSnapshotSequence = latest.sequence;
      sendState(latest);
    }
    for (observer in observers.copy()) {
      var key = observer.rawValue();
      var observerSession = observerSessions.get(key);
      if (observerHello.get(key) == true && observerSession != null)
        sendStateTo(observer, observerSession, latest);
    }
  }

  function applyBehavior(snapshot:RobotSnapshot):Void {
    var runner = behaviorRunner;
    if (runner == null || switch controlOwner { case LocalBehavior: false; case _: true; })
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
    runtime.submitPositions64(positions, nextRuntimeSequence(), snapshot.sourceTimestampNs);
    behaviorStopped = false;
  }

  function stopBehavior():Void {
    if (behaviorStopped)
      return;
    runtime.submitStop64(nextRuntimeSequence(), false);
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
      snapshot.mode, snapshot.faultCode, snapshot.receivedTimestampNs, snapshot.safety,
      snapshot.trajectoryQueueDepth, snapshot.trajectoryActive,
      snapshot.trajectoryTimeNs, snapshot.trajectoryDurationNs,
      snapshot.trajectoryTag, snapshot.trajectoryTagTimeNs,
      snapshot.sessionState, snapshot.activePlanId,
      snapshot.committedUntilNs, snapshot.queueEndTimeNs);
    sendTo(target, targetSession, RobotProtocol.state(message, targetSession, snapshot.sequence,
      snapshot.sourceTimestampNs));
    var scheduler = outbound.get(target.rawValue());
    if (scheduler == null) return;
    for (sensor in RobotSensorFrames.fromRuntimeSnapshot(snapshot)) {
      var family = sensor.image == null ? OutboundFamily.Sensor : OutboundFamily.Camera;
      if (!scheduler.shouldOffer(family, sensor.sensorId, sensor.sequence,
          NativeKit.nk_time_now_ns())) continue;
      if (sensor.image != null) {
        var image = sensor.image;
        var format = switch image.encoding {
          case "rgb8": PixelFormat.RGB8;
          case "depth32f": PixelFormat.Depth32F;
          case "jpeg": PixelFormat.JPEG;
          case _: throw 'Unsupported camera image encoding "${image.encoding}"';
        };
        var camera = new CameraFrame(snapshot.robotId, sensor.sensorId,
          sensor.kind, sensor.frameId, sensor.sequence, sensor.sourceTimestampNs,
          sensor.receivedTimestampNs, image.width, image.height, format, null,
          sensor.linkId, sensor.mountPosition.toArray(), sensor.mountRotation.toArray(),
          sensor.sourceClockId, sensor.receivedClockId);
        scheduler.offer(OutboundFamily.Camera, sensor.sensorId, sensor.sequence,
          RobotProtocol.cameraFrame(camera, image.bytes(), targetSession,
            sensor.sequence, sensor.sourceTimestampNs));
      } else {
        scheduler.offer(OutboundFamily.Sensor, sensor.sensorId, sensor.sequence,
          RobotProtocol.sensorFrame(new SensorFrameMsg(
          snapshot.robotId, sensor.sensorId, sensor.kind, sensor.frameId, sensor.sequence,
          sensor.sourceTimestampNs, sensor.receivedTimestampNs, sensor.values.toArray(),
          sensor.linkId, sensor.mountPosition.toArray(), sensor.mountRotation.toArray()),
          targetSession, sensor.sequence, sensor.sourceTimestampNs));
      }
    }
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
    if (OutboundPolicy.family(frame.messageType) != OutboundFamily.Essential)
      throw "bulk RKF1 frames must use the outbound scheduler";
    try {
      NativeTransport.send(target, frame.encode());
    } catch (_:Dynamic) {
      if (client != null && client.rawValue() == target.rawValue())
        closeClient(target);
      else closeObserver(target);
    }
  }

  function flushBulk():Void {
    var currentClient = client;
    if (currentClient != null && helloComplete)
      flushTo(currentClient, sessionId);
    for (observer in observers.copy()) {
      var key = observer.rawValue();
      var observerSession = observerSessions.get(key);
      if (observerHello.get(key) == true && observerSession != null)
        flushTo(observer, observerSession);
    }
  }

  function flushTo(target:TransportHandle, targetSession:Int64):Void {
    var scheduler = outbound.get(target.rawValue());
    if (scheduler == null) return;
    try {
      if (!scheduler.flush()) {
        if (client != null && client.rawValue() == target.rawValue()) closeClient(target);
        else closeObserver(target);
        return;
      }
      scheduler.logDrops(NativeKit.nk_time_now_ns(), targetSession);
    } catch (_:Dynamic) {
      if (client != null && client.rawValue() == target.rawValue()) closeClient(target);
      else closeObserver(target);
    }
  }

  function closeClient(currentClient:TransportHandle):Void {
    if (client != null && client.rawValue() == currentClient.rawValue()) {
      releaseRemoteOwner();
      client = null;
      helloComplete = false;
      controllerGranted = false;
      lastLeaseRenewalNs = Int64.ofInt(0);
    }
    outbound.remove(currentClient.rawValue());
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
    outbound.remove(value.rawValue());
    try { NativeTransport.close(value); } catch (_:Dynamic) {}
  }
}

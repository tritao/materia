package robotkit.client;
import nativekit.ffi.NativeKitTypes;

import nativekit.ffi.NativeKit;
import NativeKitEventValue;
import NativeKitEvents;
import NativeKitEvents.NativeKitEventSubscription;
import NativeKitRuntime;
import haxe.Int64;
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
import robotkit.protocol.SensorFrameMsg;
import robotkit.protocol.Stop;
import robotkit.transport.NativeTransport;

/**
 * Typed editor/CLI client for the RobotKit process boundary.
 *
 * The client owns only transport and protocol state. RobotRuntime state remains
 * authoritative in robotd and is delivered as immutable message values.
 */
class RobotClient {
  public final clientName:String;
  public final requestedRole:String;
  public var welcome:Null<robotkit.protocol.Welcome> = null;
  public var description:Null<RobotDescription> = null;
  public var capabilities:Null<RobotCapabilities> = null;
  public var latestState:Null<RobotStateMsg> = null;
  public var lastFault:Null<Fault> = null;
  public var stateListener:Null<RobotStateMsg->Void> = null;
  public var faultListener:Null<Fault->Void> = null;
  public var sensorListener:Null<SensorFrameMsg->Void> = null;
  public var statusListener:Null<Void->Void> = null;

  var nativeRuntime:Null<NativeKitRuntime> = null;
  var eventPump:Null<NativeKitEvents> = null;
  var owned:Null<OwnedTransportHandle> = null;
  var transport:Null<TransportHandle> = null;
  var subscription:Null<NativeKitEventSubscription> = null;
  var stream = new RobotFrameStream();
  var sessionId:Int64 = Int64.ofInt(0);
  var commandSequence:Int64 = Int64.ofInt(0);
  var lastStateSequence:Int64 = Int64.ofInt(0);
  var hasState:Bool = false;
  var connected:Bool = false;
  var failure:Null<String> = null;
  var closed:Bool = false;

  public function new(?clientName:String = "materia", ?requestedRole:String = "controller") {
    this.clientName = clientName;
    this.requestedRole = requestedRole;
  }

  /** Connects to robotd and starts the NativeKit event subscription. */
  public function connect(host:String, port:Int):Void {
    if (subscription != null || nativeRuntime != null || owned != null)
      throw "RobotKit client is already connected";
    closed = false;
    failure = null;
    stream = new RobotFrameStream();
    sessionId = Int64.ofInt(0);
    commandSequence = Int64.ofInt(0);
    lastStateSequence = Int64.ofInt(0);
    hasState = false;
    welcome = null;
    description = null;
    capabilities = null;
    latestState = null;
    lastFault = null;
    var runtime = NativeKitRuntime.start();
    var connection:Null<OwnedTransportHandle> = null;
    try {
      var connectedTransport = NativeTransport.connect(host, port);
      connection = connectedTransport;
      nativeRuntime = runtime;
      eventPump = runtime.events;
      owned = connectedTransport;
      transport = connectedTransport.borrow();
      subscription = runtime.events.listen(onEvent);
    } catch (error:Dynamic) {
      if (connection != null)
        connection.close();
      nativeRuntime = null;
      eventPump = null;
      owned = null;
      transport = null;
      subscription = null;
      runtime.dispose();
      throw error;
    }
  }

  /**
   * Connects using an event pump owned by an embedding application.
   *
   * This keeps the editor's single NativeKit initialization and event loop
   * authoritative while retaining the same typed client API.
   */
  public function connectWithEvents(host:String, port:Int, events:NativeKitEvents):Void {
    if (events == null)
      throw "RobotKit client requires a NativeKit event pump";
    if (subscription != null || nativeRuntime != null || owned != null)
      throw "RobotKit client is already connected";
    closed = false;
    failure = null;
    stream = new RobotFrameStream();
    sessionId = Int64.ofInt(0);
    commandSequence = Int64.ofInt(0);
    lastStateSequence = Int64.ofInt(0);
    hasState = false;
    welcome = null;
    description = null;
    capabilities = null;
    latestState = null;
    lastFault = null;
    var connection:Null<OwnedTransportHandle> = null;
    try {
      var connectedTransport = NativeTransport.connect(host, port);
      connection = connectedTransport;
      eventPump = events;
      owned = connectedTransport;
      transport = connectedTransport.borrow();
      subscription = events.listen(onEvent);
    } catch (error:Dynamic) {
      if (connection != null)
        connection.close();
      eventPump = null;
      owned = null;
      transport = null;
      subscription = null;
      throw error;
    }
  }

  /** Pumps all currently queued NativeKit events and reports whether any ran. */
  public function poll():Bool {
    var events = eventPump;
    if (events == null)
      throw "RobotKit client is not connected";
    var hadEvent = false;
    while (events.poll())
      hadEvent = true;
    raiseFailure();
    return hadEvent;
  }

  /** Waits for transport activity; call poll() afterwards to dispatch it. */
  public function wait(timeoutSeconds:Float):Void {
    var events = eventPump;
    if (events == null)
      throw "RobotKit client is not connected";
    events.wait(timeoutSeconds);
  }

  public function isConnected():Bool
    return connected && !closed;

  public function isReady():Bool
    return isConnected() && Int64.compare(sessionId, Int64.ofInt(0)) != 0;

  public function hasControlLease():Bool
    return isReady() && welcome != null && welcome.controlGranted;

  /** Sends one joint target through the batch command protocol. */
  public function sendJointTarget(joint:Int, mode:Int, target:Float,
      ?expiryNs:Int64):Int64 {
    var targetMode = switch mode {
      case 1: robotkit.world.JointTargetMode.Position;
      case 2: robotkit.world.JointTargetMode.Velocity;
      case 3: robotkit.world.JointTargetMode.Effort;
      case _: throw 'Unsupported joint target mode $mode';
    };
    return sendJointTargets([new robotkit.world.JointTarget(joint, targetMode, target)],
      expiryNs);
  }

  /** Sends a complete position/velocity/effort batch as one protocol frame. */
  public function sendJointTargets(targets:Array<robotkit.world.JointTarget>,
      ?expiryNs:Int64):Int64 {
    ensureReady();
    // Monotonic clocks on different hosts have no shared epoch.
    if (expiryNs != null && Int64.compare(expiryNs, Int64.ofInt(0)) != 0)
      throw "Remote absolute deadlines require clock synchronization";
    var batch = robotkit.world.JointTarget.copyBatch(targets);
    var sequence = nextCommandSequence();
    var wireTargets:Array<JointTargetValue> = [];
    for (target in batch) wireTargets.push(new JointTargetValue(target.joint,
      switch target.mode {
        case robotkit.world.JointTargetMode.Position: 1;
        case robotkit.world.JointTargetMode.Velocity: 2;
        case robotkit.world.JointTargetMode.Effort: 3;
      }, target.target));
    var value = new JointTargets(robotId(), wireTargets, sequence,
      expiryNs == null ? Int64.ofInt(0) : expiryNs);
    send(RobotProtocol.jointTargets(value, sessionId, sequence,
      NativeKit.nk_time_now_ns()));
    return sequence;
  }

  public function stop(?reason:String = "client stop", ?emergency:Bool = false):Int64 {
    ensureReady();
    var sequence = nextCommandSequence();
    var value = new Stop(robotId(), reason, emergency);
    send(RobotProtocol.stop(value, sessionId, sequence,
      NativeKit.nk_time_now_ns()));
    return sequence;
  }

  public function close():Void {
    if (closed)
      return;
    closed = true;
    connected = false;
    var statusChanged = statusListener;
    if (statusChanged != null)
      statusChanged();
    var currentSubscription = subscription;
    subscription = null;
    if (currentSubscription != null)
      currentSubscription.dispose();
    var currentOwned = owned;
    owned = null;
    transport = null;
    if (currentOwned != null)
      currentOwned.close();
    var runtime = nativeRuntime;
    nativeRuntime = null;
    eventPump = null;
    if (runtime != null)
      runtime.dispose();
  }

  function onEvent(value:NativeKitEventValue):Void switch value {
    case Raw(kind, source, _, _, _, _, _):
      var currentTransport = transport;
      if (currentTransport == null || source.rawValue() != currentTransport.rawValue())
        return;
      if (kind == EventKind.TransportConnected) {
        connected = true;
        var statusChanged = statusListener;
        if (statusChanged != null)
          statusChanged();
        send(RobotProtocol.hello(new Hello(1, clientName, "robotkit-v1", requestedRole)));
      } else if (kind == EventKind.TransportData) {
        receive(currentTransport);
      } else if (kind == EventKind.TransportClosed || kind == EventKind.TransportFailed) {
        connected = false;
        failure = "robotd closed the TCP connection";
        var statusChanged = statusListener;
        if (statusChanged != null)
          statusChanged();
      }
    case _:
  }

  function receive(currentTransport:TransportHandle):Void {
    while (true) {
      var data = NativeTransport.receive(currentTransport);
      if (data.length == 0)
        return;
      for (frame in stream.push(data))
        handleFrame(frame);
    }
  }

  function handleFrame(frame:RobotFrame):Void switch frame.messageType {
    case RobotMessageType.Welcome:
      var value = RobotProtocol.decodeWelcome(frame);
      if (Int64.compare(frame.sessionId, value.sessionId) != 0) {
        failure = "RobotKit Welcome session mismatch";
        return;
      }
      welcome = value;
      sessionId = value.sessionId;
    case RobotMessageType.RobotDescription:
      if (!validSession(frame))
        return;
      description = RobotProtocol.decodeDescription(frame);
    case RobotMessageType.RobotCapabilities:
      if (!validSession(frame))
        return;
      capabilities = RobotProtocol.decodeCapabilities(frame);
    case RobotMessageType.RobotState:
      var value = RobotProtocol.decodeState(frame);
      if (Int64.compare(frame.sessionId, sessionId) != 0)
        return;
      if (hasState && Int64.compare(value.sequence, lastStateSequence) <= 0)
        return;
      hasState = true;
      lastStateSequence = value.sequence;
      latestState = value;
      var listener = stateListener;
      if (listener != null)
        listener(value);
    case RobotMessageType.Fault:
      if (!validSession(frame))
        return;
      var value = RobotProtocol.decodeFault(frame);
      lastFault = value;
      var listener = faultListener;
      if (listener != null)
        listener(value);
      if (value.fatal)
        failure = value.message;
    case RobotMessageType.SensorFrame:
      if (!validSession(frame))
        return;
      var sensor = RobotProtocol.decodeSensorFrame(frame);
      var listener = sensorListener;
      if (listener != null)
        listener(sensor);
    case _:
  }

  function nextCommandSequence():Int64 {
    commandSequence = Int64.add(commandSequence, Int64.ofInt(1));
    return commandSequence;
  }

  function robotId():Int64 {
    var value = welcome;
    if (value == null)
      throw "RobotKit client has not received Welcome";
    return value.robotId;
  }

  function validSession(frame:RobotFrame):Bool
    return Int64.compare(frame.sessionId, sessionId) == 0
      && Int64.compare(sessionId, Int64.ofInt(0)) != 0;

  function ensureReady():Void {
    if (!isReady())
      throw "RobotKit client is not ready; wait for Welcome first";
    if (!hasControlLease())
      throw "RobotKit client does not hold the control lease";
    raiseFailure();
  }

  function raiseFailure():Void {
    var value = failure;
    if (value != null)
      throw value;
  }

  function send(frame:RobotFrame):Void {
    var currentTransport = transport;
    if (currentTransport == null)
      throw "RobotKit client is not connected";
    NativeTransport.send(currentTransport, frame.encode());
  }
}

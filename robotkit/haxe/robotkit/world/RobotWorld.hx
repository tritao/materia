package robotkit.world;

import sys.thread.Mutex;

/**
 * Owns the logical robot composition for one Materia world.
 *
 * The world deliberately stores only the `Robot` boundary. A remote
 * robot and a simulated robot therefore participate in the same snapshots,
 * command routing, lifecycle, and change notifications without the world
 * knowing which transport or physics backend supplies them.
 */
class RobotWorld {
  final robotMap:Map<RobotId, Robot>;
  final pendingEvents:Array<RobotWorldEvent> = [];
  final eventMutex = new Mutex();
  final observers:Array<RobotWorldEvent->Void> = [];
  var pendingEventCountValue:Int = 0;

  var sequence:Int = 0;
  var topologyRevision:Int = 0;
  var closed:Bool = false;

  public function new() {
    robotMap = new Map<RobotId, Robot>();
  }

  /** Takes ownership of the robot until detach() or close(). */
  public function attach(robot:Robot):Void {
    ensureOpen();
    if (robot == null) throw "RobotWorld cannot attach a null robot";
    robot.setChangeListener(onRobotChanged);
    try {
      if (robotMap.exists(robot.id()))
        throw 'RobotWorld already contains "${robot.id()}"';
      robotMap.set(robot.id(), robot);
    } catch (error:Dynamic) {
      robot.setChangeListener(null);
      throw error;
    }
    topologyRevision++;
    sequence++;
    enqueue(RobotWorldEvent.RobotAttached(robot.id()));
  }

  /** Returns the robot currently attached under an ID, without transferring ownership. */
  public function robot(id:RobotId):Null<Robot> return robotMap.get(id);

  /** Returns attached IDs in deterministic lexical order. */
  public function robotIds():Array<RobotId> {
    var result:Array<RobotId> = [];
    for (id in robotMap.keys()) result.push(id);
    result.sort(function(left, right) return Reflect.compare(left, right));
    return result;
  }

  /** Returns a copy of the attached robot collection. */
  public function robots():Array<Robot> return allRobots();

  /** Detaches and returns a robot without closing it. */
  public function detach(id:RobotId):Null<Robot> {
    ensureOpen();
    var robot = robotMap.get(id);
    if (robot == null) return null;
    robotMap.remove(id);
    robot.setChangeListener(null);
    topologyRevision++;
    sequence++;
    enqueue(RobotWorldEvent.RobotDetached(id));
    return robot;
  }

  /** Observes lifecycle and adapter changes; callbacks run during pump(). */
  public function subscribe(listener:RobotWorldEvent->Void):RobotWorldSubscription {
    ensureOpen();
    if (listener == null) throw "RobotWorld subscription requires a listener";
    observers.push(listener);
    return new RobotWorldSubscription(function() {
      var index = observers.indexOf(listener);
      if (index >= 0) observers.splice(index, 1);
    });
  }

  /** Returns descriptions in deterministic logical-ID order. */
  public function discover():Array<RobotDescription> {
    ensureOpen();
    var result:Array<RobotDescription> = [];
    for (robot in allRobots()) result.push(robot.description());
    return result;
  }

  public function health():RobotHealthSummary {
    ensureOpen();
    var ready = 0;
    var connecting = 0;
    var disconnected = 0;
    var faulted = 0;
    for (robot in allRobots()) switch robot.status() {
      case Ready: ready++;
      case Connecting: connecting++;
      case Disconnected: disconnected++;
      case Fault: faulted++;
    }
    var total = 0;
    for (_ in robotMap.keys()) total++;
    return new RobotHealthSummary(total, ready, connecting, disconnected, faulted);
  }

  /** Copies all robot observations into one deterministic world snapshot. */
  public function snapshot():WorldSnapshot {
    ensureOpen();
    pump();
    var source = new Map<RobotId, RobotSnapshot>();
    var sourceTimestampNs = haxe.Int64.ofInt(0);
    var receivedTimestampNs = haxe.Int64.ofInt(0);
    for (robot in allRobots()) {
      var value = robot.snapshot();
      source.set(robot.id(), value);
      if (haxe.Int64.compare(value.sourceTimestampNs, sourceTimestampNs) > 0)
        sourceTimestampNs = value.sourceTimestampNs;
      if (haxe.Int64.compare(value.receivedTimestampNs, receivedTimestampNs) > 0)
        receivedTimestampNs = value.receivedTimestampNs;
    }
    // Some adapters discover a new sample while they are being observed. Apply
    // that notification before publishing the snapshot so its sequence describes
    // the state that was actually returned.
    pump();
    return new WorldSnapshot(sequence, topologyRevision, sourceTimestampNs, source,
      receivedTimestampNs);
  }

  /** Enqueues an adapter notification for application by the world owner. */
  public function enqueue(event:RobotWorldEvent):Void {
    if (closed) return;
    if (event == null) throw "RobotWorld cannot enqueue a null event";
    eventMutex.acquire();
    pendingEvents.push(event);
    pendingEventCountValue++;
    eventMutex.release();
  }

  /** Applies queued adapter events on the world owner's event-loop turn. */
  public function pump():Int {
    ensureOpen();
    var applied = 0;
    while (true) {
      eventMutex.acquire();
      var event:Null<RobotWorldEvent> = pendingEvents.length == 0 ? null : pendingEvents.shift();
      if (event != null) pendingEventCountValue--;
      eventMutex.release();
      if (event == null) break;
      switch event {
        case RobotChanged(id):
          if (robotMap.exists(id)) {
            sequence++;
            applied++;
          }
        case RobotAttached(_):
        case RobotDetached(_):
      }
      publish(event);
    }
    return applied;
  }

  public function pendingEventCount():Int {
    eventMutex.acquire();
    var result = pendingEventCountValue;
    eventMutex.release();
    return result;
  }

  /** Routes one command to the selected robot adapter. */
  public function submit(id:RobotId, command:RobotCommand):Void {
    ensureOpen();
    requireRobot(id).submit(command);
  }

  /** Routes a normal or emergency stop to the selected robot adapter. */
  public function stop(id:RobotId, mode:StopMode):Void {
    ensureOpen();
    requireRobot(id).stop(mode);
  }

  public function status():RobotStatus {
    if (robotMap.keys().hasNext() == false) return Disconnected;
    var hasConnecting = false;
    for (robot in allRobots()) {
      var robotStatus = robot.status();
      switch (robotStatus) {
        case Fault:
          return Fault;
        case Ready:
        case Connecting:
          hasConnecting = hasConnecting || robotStatus == Connecting;
        case Disconnected:
          return Connecting;
      }
    }
    return hasConnecting ? Connecting : Ready;
  }

  /** Closes adapters while leaving externally owned Simulation objects alive. */
  public function close():Void {
    if (closed) return;
    closed = true;
    for (robot in allRobots()) {
      robot.setChangeListener(null);
      robot.close();
    }
    for (id in robotIds()) robotMap.remove(id);
    observers.resize(0);
  }

  function onRobotChanged(id:RobotId):Void {
    enqueue(RobotWorldEvent.RobotChanged(id));
  }

  function publish(event:RobotWorldEvent):Void {
    for (observer in observers.copy()) observer(event);
  }

  function ensureOpen():Void {
    if (closed) throw "RobotWorld has been closed";
  }

  function requireRobot(id:RobotId):Robot {
    var robot = robotMap.get(id);
    if (robot == null) throw 'RobotWorld does not contain "$id"';
    return robot;
  }

  function allRobots():Array<Robot> {
    var result:Array<Robot> = [];
    for (id in robotIds()) result.push(robotMap.get(id));
    return result;
  }
}

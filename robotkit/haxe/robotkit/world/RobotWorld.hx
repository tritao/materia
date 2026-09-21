package robotkit.world;

/**
 * Owns the logical robot composition for one Materia world.
 *
 * The world deliberately stores only the `RobotInstance` boundary. A remote
 * robot and a simulated robot therefore participate in the same snapshots,
 * command routing, lifecycle, and change notifications without the world
 * knowing which transport or physics backend supplies them.
 */
class RobotWorld {
  public final robots:Map<RobotId, RobotInstance>;

  var sequence:Int = 0;
  var topologyRevision:Int = 0;
  var closed:Bool = false;

  public function new() {
    robots = new Map<RobotId, RobotInstance>();
  }

  /** Takes ownership of the instance until detach() or close(). */
  public function attach(robot:RobotInstance):Void {
    ensureOpen();
    if (robot == null) throw "RobotWorld cannot attach a null robot";
    robot.setChangeListener(onRobotChanged);
    try {
      if (robots.exists(robot.id()))
        throw 'RobotWorld already contains "${robot.id()}"';
      robots.set(robot.id(), robot);
    } catch (error:Dynamic) {
      robot.setChangeListener(null);
      throw error;
    }
    topologyRevision++;
    sequence++;
  }

  public function detach(id:RobotId):Null < RobotInstance > {
    ensureOpen();
    var robot = robots.get(id);
    if (robot == null) return null;
    robots.remove(id);
    robot.setChangeListener(null);
    topologyRevision++;
    sequence++;
    return robot;
  }

  /** Copies all robot observations into one deterministic world snapshot. */
  public function snapshot():WorldSnapshot {
    ensureOpen();
    var source = new Map<RobotId, RobotSnapshot>();
    var timestampNs = haxe.Int64.ofInt(0);
    for (robot in allRobots()) {
      var value = robot.snapshot();
      source.set(robot.id(), value);
      if (haxe.Int64.compare(value.timestampNs, timestampNs) > 0) timestampNs = value.timestampNs;
    }
    return new WorldSnapshot(sequence, topologyRevision, timestampNs, source);
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
    if (robots.keys().hasNext() == false) return Disconnected;
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
    for (id in robotIds()) robots.remove(id);
  }

  function onRobotChanged():Void {
    if (!closed) sequence++;
  }

  function ensureOpen():Void {
    if (closed) throw "RobotWorld has been closed";
  }

  function requireRobot(id:RobotId):RobotInstance {
    var robot = robots.get(id);
    if (robot == null) throw 'RobotWorld does not contain "$id"';
    return robot;
  }

  function robotIds():Array<RobotId> {
    var result:Array<RobotId> = [];
    for (id in robots.keys()) result.push(id);
    result.sort(function(left, right) return Reflect.compare(left, right));
    return result;
  }

  function allRobots():Array<RobotInstance> {
    var result:Array<RobotInstance> = [];
    for (id in robotIds()) result.push(robots.get(id));
    return result;
  }
}

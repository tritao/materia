package robotkit.world;

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
  }

  function onRobotChanged():Void {
    if (!closed) sequence++;
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

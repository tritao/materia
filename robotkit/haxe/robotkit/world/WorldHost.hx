package robotkit.world;

/** Pure world orchestration, embeddable in Materia or a future worldd. */
class WorldHost {
  public final robots:RobotRegistry;

  var sequence:Int = 0;
  var topologyRevision:Int = 0;
  var closed:Bool = false;

  public function new() {
    robots = new RobotRegistry();
  }

  /** Takes ownership of the instance until detach() or close(). */
  public function attach(robot:RobotInstance):Void {
    ensureOpen();
    if (robot == null) throw "WorldHost cannot attach a null robot";
    robot.setChangeListener(onRobotChanged);
    try {
      robots.add(robot);
    } catch (error:Dynamic) {
      robot.setChangeListener(null);
      throw error;
    }
    topologyRevision++;
    sequence++;
  }

  public function detach(id:RobotId):Null < RobotInstance > {
    ensureOpen();
    var robot = robots.remove(id);
    if (robot == null) return null;
    robot.setChangeListener(null);
    topologyRevision++;
    sequence++;
    return robot;
  }

  public function snapshot():WorldSnapshot {
    ensureOpen();
    var source = new Map<RobotId, RobotSnapshot>();
    var timestampNs = haxe.Int64.ofInt(0);
    for (robot in robots.all()) {
      var value = robot.snapshot();
      source.set(robot.id(), value);
      if (haxe.Int64.compare(value.timestampNs, timestampNs) > 0) timestampNs = value.timestampNs;
    }
    return new WorldSnapshot(sequence, topologyRevision, timestampNs, source);
  }

  public function submit(id:RobotId, command:RobotCommand):Void {
    ensureOpen();
    requireRobot(id).submit(command);
  }

  public function stop(id:RobotId, mode:StopMode):Void {
    ensureOpen();
    requireRobot(id).stop(mode);
  }

  public function status():RobotStatus {
    if (robots.ids().length == 0) return Disconnected;
    var hasConnecting = false;
    for (robot in robots.all()) {
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

  public function close():Void {
    if (closed) return;
    closed = true;
    for (robot in robots.all()) {
      robot.setChangeListener(null);
      robot.close();
    }
    for (id in robots.ids()) robots.remove(id);
  }

  function onRobotChanged():Void {
    if (!closed) sequence++;
  }

  function ensureOpen():Void {
    if (closed) throw "WorldHost has been closed";
  }

  function requireRobot(id:RobotId):RobotInstance {
    var robot = robots.get(id);
    if (robot == null) throw 'WorldHost does not contain "$id"';
    return robot;
  }
}

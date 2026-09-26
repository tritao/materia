package robotkit.world;

import haxe.Int64;

/** In-memory deterministic log used by tests, diagnostics, and replay tools. */
class RobotRecording implements RobotRecordingSink {
  public final commands:Array<RobotCommand> = [];
  public final snapshots:Array<RobotSnapshot> = [];
  public final faults:Array<RobotFault> = [];
  public final worlds:Array<WorldSnapshot> = [];
  public final events:Array<RobotRecordingEvent> = [];
  public final entries:Array<RobotRecordingEntry> = [];
  var nextOrdinal:Int64 = Int64.ofInt(0);

  public function new() {}

  public function recordCommand(command:RobotCommand, ?robotId:RobotId = ""):Void {
    switch command {
      case JointTargets(targets, expiryNs):
        var copy = RobotCommand.JointTargets(JointTarget.copyBatch(targets), expiryNs);
        commands.push(copy);
        events.push(RobotRecordingEvent.Command(copy));
        append(RobotRecordingEvent.Command(copy), robotId);
      case TrajectoryChunk(chunk):
        var copy = RobotCommand.TrajectoryChunk(chunk.copy());
        commands.push(copy);
        events.push(RobotRecordingEvent.Command(copy));
        append(RobotRecordingEvent.Command(copy), robotId);
    }
  }

  public function recordSnapshot(snapshot:RobotSnapshot):Void {
    var copy = copyRobotSnapshot(snapshot);
    snapshots.push(copy);
    events.push(RobotRecordingEvent.RobotSnapshot(copyRobotSnapshot(copy)));
    append(RobotRecordingEvent.RobotSnapshot(copyRobotSnapshot(copy)), copy.id);
  }

  public function recordFault(fault:RobotFault):Void {
    var copy = new RobotFault(fault.id, fault.code, fault.message, fault.fatal);
    faults.push(copy);
    events.push(RobotRecordingEvent.Fault(
      new RobotFault(copy.id, copy.code, copy.message, copy.fatal)));
    append(RobotRecordingEvent.Fault(
      new RobotFault(copy.id, copy.code, copy.message, copy.fatal)), copy.id);
  }

  public function recordWorld(snapshot:WorldSnapshot):Void {
    var source = new Map<RobotId, RobotSnapshot>();
    for (id in snapshot.robotIds()) {
      var robot = snapshot.robot(id);
      if (robot != null) source.set(id, robot);
    }
    var copy = new WorldSnapshot(snapshot.sequence, snapshot.topologyRevision,
      snapshot.sourceTimestampNs, source, snapshot.receivedTimestampNs);
    worlds.push(copy);
    events.push(RobotRecordingEvent.World(copy));
    append(RobotRecordingEvent.World(copy), "");
  }

  public function recordSensor(robotId:RobotId, sensor:SensorFrame):Void {
    var copy = sensor.copy();
    events.push(RobotRecordingEvent.Sensor(robotId, copy));
    append(RobotRecordingEvent.Sensor(robotId, copy.copy()), robotId);
  }

  /** Records one owner-thread world notification in arrival order. */
  public function recordEvent(event:RobotWorldEvent):Void switch event {
    case RobotAttached(id): pushWorldEvent(RobotAttached(id), id);
    case RobotDetached(id): pushWorldEvent(RobotDetached(id), id);
    case RobotChanged(id): pushWorldEvent(RobotChanged(id), id);
  }

  /** Subscribes this recording to lifecycle and adapter events. */
  public function attach(world:RobotWorld):RobotWorldSubscription {
    if (world == null) throw "RobotRecording requires a RobotWorld";
    return world.subscribe(recordEvent);
  }

  public static function copyRobotSnapshot(value:RobotSnapshot):RobotSnapshot
    return new RobotSnapshot(value.id, value.sourceSequence, value.sourceTimestampNs,
      value.positions.toArray(), value.velocities.toArray(), value.efforts.toArray(),
      value.mode, value.faultCode, value.receivedTimestampNs,
      value.sensors.toArray(), value.sourceClockId, value.receivedClockId,
      value.safety);

  function pushWorldEvent(event:RobotWorldEvent, robotId:RobotId):Void {
    events.push(RobotRecordingEvent.WorldEvent(event));
    append(RobotRecordingEvent.WorldEvent(event), robotId);
  }

  function append(event:RobotRecordingEvent, robotId:RobotId):Void {
    var sequence = Int64.ofInt(0);
    var timestamp = Int64.ofInt(0);
    var clock = "unspecified";
    switch event {
      case RobotSnapshot(value): sequence = value.sourceSequence; timestamp = value.sourceTimestampNs; clock = value.sourceClockId;
      case Sensor(_, value): sequence = value.sequence; timestamp = value.sourceTimestampNs; clock = value.sourceClockId;
      case _: // Event-specific fields remain zero when the source contract has none.
    }
    entries.push(new RobotRecordingEntry(nextOrdinal, robotId, event, sequence, timestamp, clock));
    nextOrdinal = Int64.add(nextOrdinal, Int64.ofInt(1));
  }

  /** Used by persistent readers after schema validation. */
  public function ingest(entry:RobotRecordingEntry):Void {
    if (entry == null) throw "Cannot ingest a null recording entry";
    entries.push(entry);
    events.push(entry.event);
    switch entry.event {
      case Command(value): commands.push(value);
      case RobotSnapshot(value): snapshots.push(copyRobotSnapshot(value));
      case Fault(value): faults.push(new RobotFault(value.id, value.code, value.message, value.fatal));
      case World(value): worlds.push(value);
      case Sensor(_, _), WorldEvent(_):
    }
    if (Int64.compare(entry.ordinal, nextOrdinal) >= 0)
      nextOrdinal = Int64.add(entry.ordinal, Int64.ofInt(1));
  }
}

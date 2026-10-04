package robotkit.recording;

import robotkit.core.JointTarget;
import robotkit.core.Robot;
import robotkit.core.RobotCommand;
import robotkit.core.RobotEvent;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.SensorFrame;
import robotkit.execution.FiredProcessEvent;
import robotkit.world.RobotWorld;
import robotkit.world.RobotWorldEvent;
import robotkit.world.RobotWorldSubscription;
import robotkit.world.WorldSnapshot;

import haxe.Int64;

/** In-memory deterministic log used by tests, diagnostics, and replay tools. */
class RobotRecording implements RobotRecordingSink {
  public final commands:Array<RobotCommand> = [];
  public final snapshots:Array<RobotSnapshot> = [];
  public final faults:Array<RobotFault> = [];
  public final worlds:Array<WorldSnapshot> = [];
  public final processEvents:Array<FiredProcessEvent> = [];
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
      case ExecutionPlan(plan):
        var copy = RobotCommand.ExecutionPlan(plan.copy());
        commands.push(copy);
        events.push(RobotRecordingEvent.Command(copy));
        append(RobotRecordingEvent.Command(copy), robotId);
      case Hold | Resume | Abort:
        commands.push(command);
        events.push(RobotRecordingEvent.Command(command));
        append(RobotRecordingEvent.Command(command), robotId);
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

  public function recordProcessEvent(robotId:RobotId, value:FiredProcessEvent):Void {
    if (value == null) throw "Process record is required";
    processEvents.push(value);
    events.push(RobotRecordingEvent.ProcessEvent(value));
    append(RobotRecordingEvent.ProcessEvent(value), robotId);
  }

  public function recordRobotEvent(robotId:RobotId, value:RobotEvent):Void {
    if (value == null) throw "Robot event is required";
    switch value {
      case Observation(_, _): recordChannel(robotId, "perception.image_detections", value);
      case Overflow(_, _): // The next observation's ordinal preserves this gap.
    }
  }

  public function recordChannel(robotId:RobotId, name:String, payload:Dynamic):Void {
    if (name == null || name.length == 0 || payload == null)
      throw "Recording channel requires a name and payload";
    var event = RobotRecordingEvent.Channel(robotId, name, payload);
    events.push(event);
    append(event, robotId);
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
      value.safety, value.trajectoryQueueDepth, value.trajectoryActive,
      value.trajectoryTimeNs, value.trajectoryDurationNs,
      value.trajectoryTag, value.trajectoryTagTimeNs,
      value.sessionState, value.activePlanId,
      value.committedUntilNs, value.queueEndTimeNs, value.setpointPositions.toArray(), value.streamSequences);

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
      case ProcessEvent(value): timestamp = value.appliedOwnerTimeNs; clock = "runtime-owner";
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
      case ProcessEvent(value): processEvents.push(value);
      case Sensor(_, _), WorldEvent(_), Channel(_, _, _):
    }
    if (Int64.compare(entry.ordinal, nextOrdinal) >= 0)
      nextOrdinal = Int64.add(entry.ordinal, Int64.ofInt(1));
  }
}

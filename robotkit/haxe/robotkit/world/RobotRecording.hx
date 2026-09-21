package robotkit.world;

import haxe.Int64;

/** In-memory deterministic log used by tests, diagnostics, and replay tools. */
class RobotRecording {
  public final commands:Array<RobotCommand> = [];
  public final snapshots:Array<RobotSnapshot> = [];
  public final faults:Array<RobotFault> = [];
  public final worlds:Array<WorldSnapshot> = [];
  public final events:Array<RobotRecordingEvent> = [];

  public function new() {}

  public function recordCommand(command:RobotCommand):Void {
    switch command {
      case JointPosition(joint, target, expiryNs):
        var copy = RobotCommand.JointPosition(joint, target, expiryNs);
        commands.push(copy);
        events.push(RobotRecordingEvent.Command(copy));
    }
  }

  public function recordSnapshot(snapshot:RobotSnapshot):Void {
    var copy = copyRobotSnapshot(snapshot);
    snapshots.push(copy);
    events.push(RobotRecordingEvent.Snapshot(copyRobotSnapshot(copy)));
  }

  public function recordFault(fault:RobotFault):Void {
    var copy = new RobotFault(fault.id, fault.code, fault.message, fault.fatal);
    faults.push(copy);
    events.push(RobotRecordingEvent.Fault(
      new RobotFault(copy.id, copy.code, copy.message, copy.fatal)));
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
  }

  /** Records one owner-thread world notification in arrival order. */
  public function recordEvent(event:RobotWorldEvent):Void switch event {
    case RobotAttached(id): events.push(RobotRecordingEvent.WorldEvent(RobotAttached(id)));
    case RobotDetached(id): events.push(RobotRecordingEvent.WorldEvent(RobotDetached(id)));
    case RobotChanged(id): events.push(RobotRecordingEvent.WorldEvent(RobotChanged(id)));
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
      value.sensors.toArray());
}

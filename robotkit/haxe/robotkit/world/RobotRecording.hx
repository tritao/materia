package robotkit.world;

import haxe.Int64;

/** In-memory deterministic log used by tests, diagnostics, and replay tools. */
class RobotRecording {
  public final commands:Array<RobotCommand> = [];
  public final snapshots:Array<RobotSnapshot> = [];
  public final faults:Array<RobotFault> = [];
  public final worlds:Array<WorldSnapshot> = [];

  public function new() {}

  public function recordCommand(command:RobotCommand):Void {
    switch command {
      case JointPosition(joint, target, expiryNs):
        commands.push(RobotCommand.JointPosition(joint, target, expiryNs));
    }
  }

  public function recordSnapshot(snapshot:RobotSnapshot):Void
    snapshots.push(copyRobotSnapshot(snapshot));

  public function recordFault(fault:RobotFault):Void
    faults.push(new RobotFault(fault.id, fault.code, fault.message, fault.fatal));

  public function recordWorld(snapshot:WorldSnapshot):Void {
    var source = new Map<RobotId, RobotSnapshot>();
    for (id in snapshot.robotIds()) {
      var robot = snapshot.robot(id);
      if (robot != null) source.set(id, robot);
    }
    worlds.push(new WorldSnapshot(snapshot.sequence, snapshot.topologyRevision,
      snapshot.sourceTimestampNs, source, snapshot.receivedTimestampNs));
  }

  public static function copyRobotSnapshot(value:RobotSnapshot):RobotSnapshot
    return new RobotSnapshot(value.id, value.sourceSequence, value.sourceTimestampNs,
      value.positions.toArray(), value.velocities.toArray(), value.efforts.toArray(),
      value.mode, value.faultCode, value.receivedTimestampNs,
      value.sensors.toArray());
}

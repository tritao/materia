package robotkit.world;

/** Robot adapter that deterministically replays one recorded robot. */
class ReplayRobot implements Robot {
  public final logicalId:RobotId;
  /** Commands produced while replaying; never written into the source recording. */
  public final generatedCommands:RobotRecording;
  final descriptionValue:RobotDescription;
  final capabilitiesValue:RobotCapabilities;
  final source:Array<ReplayObservation> = [];
  var index:Int = 0;
  var listener:Null<RobotId->Void> = null;
  var closed:Bool = false;

  public function new(id:RobotId, recording:RobotRecording,
      ?description:RobotDescription, ?capabilities:RobotCapabilities,
      ?generatedCommands:RobotRecording) {
    if (id == null || id.length == 0) throw "ReplayRobot requires a non-empty recorded robot ID";
    if (recording == null) throw "ReplayRobot requires a recording";
    logicalId = id;
    this.generatedCommands = generatedCommands == null ? new RobotRecording() : generatedCommands;
    buildTimeline(recording);
    descriptionValue = description == null ? new RobotDescription(id, id, [], []) : description;
    capabilitiesValue = capabilities == null
      ? new RobotCapabilities(id, source.length == 0 ? 0 : source[0].snapshot.positions.length,
          true, false, false, false) : capabilities;
  }

  public function id():RobotId return logicalId;
  public function status():RobotStatus return closed ? Disconnected : Ready;
  public function description():RobotDescription return descriptionValue;
  public function capabilities():RobotCapabilities return capabilitiesValue;

  public function snapshot():RobotSnapshot {
    ensureLive();
    return source.length == 0 ? emptySnapshot() : RobotRecording.copyRobotSnapshot(source[index].snapshot);
  }

  public function sensors():Array<SensorFrame> return snapshot().sensors.toArray();

  public function fault():Null<RobotFault> {
    ensureLive();
    if (source.length == 0 || source[index].fault == null) return null;
    var value:RobotFault = cast source[index].fault;
    return new RobotFault(value.id, value.code, value.message, value.fatal);
  }

  public function submit(command:RobotCommand):Void {
    ensureLive();
    generatedCommands.recordCommand(command, logicalId);
  }

  public function stop(mode:StopMode):Void ensureLive();
  public function resetSafety():Void ensureLive();

  /** Advances one selected robot observation in recording ordinal order. */
  public function advance():Bool {
    if (closed || source.length == 0 || index + 1 >= source.length) return false;
    index++;
    var value = listener;
    if (value != null) value(logicalId);
    return true;
  }

  public function setChangeListener(value:Null<RobotId->Void>):Void listener = value;
  public function close():Void { closed = true; listener = null; }

  function buildTimeline(recording:RobotRecording):Void {
    var current = emptySnapshot();
    var currentFault:Null<RobotFault> = null;
    for (entry in recording.entries) switch entry.event {
      case RobotSnapshot(value) if (value.id == logicalId):
        current = RobotRecording.copyRobotSnapshot(value);
        if (value.faultCode == 0) currentFault = null;
        source.push(new ReplayObservation(current, currentFault));
      case Sensor(robotId, value) if (robotId == logicalId):
        var sensors = current.sensors.toArray();
        var replaced = false;
        for (sensorIndex in 0...sensors.length) if (sensors[sensorIndex].sensorId == value.sensorId) {
          sensors[sensorIndex] = value.copy(); replaced = true; break;
        }
        if (!replaced) sensors.push(value.copy());
        current = copyWith(current, sensors, current.faultCode);
        source.push(new ReplayObservation(current, currentFault));
      case Fault(value) if (value.id == logicalId):
        currentFault = new RobotFault(value.id, value.code, value.message, value.fatal);
        current = copyWith(current, current.sensors.toArray(), value.code);
        source.push(new ReplayObservation(current, currentFault));
      case World(value):
        var selected = value.robot(logicalId);
        if (selected != null) {
          current = RobotRecording.copyRobotSnapshot(selected);
          if (current.faultCode == 0) currentFault = null;
          source.push(new ReplayObservation(current, currentFault));
        }
      case Command(_), WorldEvent(_), RobotSnapshot(_), Sensor(_, _), Fault(_):
    }
  }

  function emptySnapshot():RobotSnapshot return new RobotSnapshot(logicalId,
    haxe.Int64.ofInt(0), haxe.Int64.ofInt(0), [], [], [], 0, 0);

  static function copyWith(value:RobotSnapshot, sensors:Array<SensorFrame>, faultCode:Int):RobotSnapshot
    return new RobotSnapshot(value.id, value.sourceSequence, value.sourceTimestampNs,
      value.positions.toArray(), value.velocities.toArray(), value.efforts.toArray(),
      value.mode, faultCode, value.receivedTimestampNs, sensors,
      value.sourceClockId, value.receivedClockId, value.safety,
      value.trajectoryQueueDepth, value.trajectoryActive,
      value.trajectoryTimeNs, value.trajectoryDurationNs);

  function ensureLive():Void if (closed) throw "ReplayRobot has been closed";
}

private class ReplayObservation {
  public final snapshot:RobotSnapshot;
  public final fault:Null<RobotFault>;
  public function new(snapshot:RobotSnapshot, fault:Null<RobotFault>) {
    this.snapshot = RobotRecording.copyRobotSnapshot(snapshot);
    this.fault = fault == null ? null : new RobotFault(fault.id, fault.code, fault.message, fault.fatal);
  }
}

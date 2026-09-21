package robotkit.world;

/** Robot adapter that replays recorded snapshots through the normal Robot boundary. */
class ReplayRobot implements Robot {
  public final logicalId:RobotId;
  final descriptionValue:RobotDescription;
  final capabilitiesValue:RobotCapabilities;
  final source:Array<RobotSnapshot>;
  final recording:RobotRecording;
  var index:Int = 0;
  var listener:Null<RobotId->Void> = null;
  var closed:Bool = false;

  public function new(id:RobotId, recording:RobotRecording,
      ?description:RobotDescription, ?capabilities:RobotCapabilities) {
    if (id == null || id.length == 0) throw "ReplayRobot requires a non-empty logical ID";
    logicalId = id;
    this.recording = recording;
    source = [];
    for (snapshot in recording.snapshots)
      source.push(RobotRecording.copyRobotSnapshot(snapshot));
    descriptionValue = description == null
      ? new RobotDescription(id, id, [], []) : description;
    capabilitiesValue = capabilities == null
      ? new RobotCapabilities(id, 0, true, false, false, false) : capabilities;
  }

  public function id():RobotId return logicalId;
  public function status():RobotStatus return closed ? Disconnected : Ready;
  public function description():RobotDescription return descriptionValue;
  public function capabilities():RobotCapabilities return capabilitiesValue;

  public function snapshot():RobotSnapshot {
    if (closed) throw "ReplayRobot has been closed";
    if (source.length == 0)
      return new RobotSnapshot(logicalId, haxe.Int64.ofInt(0), haxe.Int64.ofInt(0),
        [], [], [], 0, 0);
    return RobotRecording.copyRobotSnapshot(source[index]);
  }

  public function sensors():Array<SensorFrame> return snapshot().sensors.toArray();
  public function fault():Null<RobotFault> return null;

  public function submit(command:RobotCommand):Void {
    if (closed) throw "ReplayRobot has been closed";
    recording.recordCommand(command);
  }

  public function stop(mode:StopMode):Void {
    if (closed) throw "ReplayRobot has been closed";
  }

  /** Advances one recorded source sample and notifies the owning RobotWorld. */
  public function advance():Bool {
    if (closed || source.length == 0 || index + 1 >= source.length) return false;
    index++;
    var value = listener;
    if (value != null) value(logicalId);
    return true;
  }

  public function setChangeListener(value:Null<RobotId->Void>):Void listener = value;
  public function close():Void {
    closed = true;
    listener = null;
  }
}

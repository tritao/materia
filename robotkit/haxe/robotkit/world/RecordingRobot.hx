package robotkit.world;

/** Decorates any Robot and records its accepted target batches and observations. */
class RecordingRobot implements Robot {
  public final source:Robot;
  public final recording:RobotRecordingSink;
  /** First logging failure; robot operations continue and callers can inspect this value. */
  public var recordingError(default, null):Null<String> = null;

  var lastRecordedFaultCode:Int = 0;

  public function new(source:Robot, recording:RobotRecordingSink) {
    if (source == null || recording == null)
      throw "RecordingRobot requires a source robot and recording sink";
    this.source = source;
    this.recording = recording;
  }

  public function id():RobotId return source.id();
  public function status():RobotStatus return source.status();
  public function description():RobotDescription return source.description();
  public function capabilities():RobotCapabilities return source.capabilities();

  public function snapshot():RobotSnapshot {
    var observation = source.snapshot();
    record(function() recording.recordSnapshot(observation));
    if (observation.faultCode == 0) {
      lastRecordedFaultCode = 0;
    } else if (observation.faultCode != lastRecordedFaultCode) {
      var currentFault = source.fault();
      if (currentFault != null) record(function() recording.recordFault(currentFault));
      lastRecordedFaultCode = observation.faultCode;
    }
    return observation;
  }

  public function sensors():Array<SensorFrame> return source.sensors();
  public function fault():Null<RobotFault> return source.fault();

  public function submit(command:RobotCommand):Void {
    source.submit(command);
    record(function() recording.recordCommand(command, source.id()));
  }

  public function stop(mode:StopMode):Void source.stop(mode);

  public function setChangeListener(listener:Null<RobotId->Void>):Void
    source.setChangeListener(listener);

  public function close():Void {
    source.setChangeListener(null);
    source.close();
  }

  function record(action:Void->Void):Void {
    try action() catch (error:Dynamic) {
      if (recordingError == null) recordingError = Std.string(error);
    }
  }
}

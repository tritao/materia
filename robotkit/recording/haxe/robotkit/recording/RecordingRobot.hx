package robotkit.recording;

import robotkit.core.Robot;
import robotkit.core.RobotCapabilities;
import robotkit.core.RobotCommand;
import robotkit.core.RobotDescription;
import robotkit.core.RobotEvent;
import robotkit.core.RobotEventRing;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.RobotStatus;
import robotkit.core.SensorFrame;
import robotkit.core.StopMode;

/** Decorates any Robot and records its accepted target batches and observations. */
class RecordingRobot implements Robot {
  public final source:Robot;
  public final recording:RobotRecordingSink;
  /** First logging failure; robot operations continue and callers can inspect this value. */
  public var recordingError(default, null):Null<String> = null;

  var lastRecordedFaultCode:Int = 0;
  var lastRecordedEventOrdinal:haxe.Int64 = haxe.Int64.ofInt(0);

  var streamRecording:robotkit.streams.SensorStreamSubscription;

  public function new(source:Robot, recording:RobotRecordingSink) {
    if (source == null || recording == null)
      throw "RecordingRobot requires a source robot and recording sink";
    this.source = source;
    this.recording = recording;
    streamRecording = source.streams().subscribe("*", function(sample) {
      if (sample.frame != null) record(function() recording.recordSensor(source.id(), sample.frame));
    });
  }

  public function id():RobotId return source.id();
  public function status():RobotStatus return source.status();
  public function description():RobotDescription return source.description();
  public function capabilities():RobotCapabilities return source.capabilities();

  public function snapshot():RobotSnapshot {
    captureEvents();
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

  public function streams():robotkit.streams.SensorStreams return source.streams();
  public function events(afterOrdinal:haxe.Int64, max:Int):Array<RobotEvent> {
    captureEvents();
    return source.events(afterOrdinal, max);
  }

  function captureEvents():Void {
    var available:Array<RobotEvent>;
    do {
      available = source.events(lastRecordedEventOrdinal, 256);
      for (event in available) {
        lastRecordedEventOrdinal = RobotEventRing.ordinalOf(event);
        record(function() recording.recordRobotEvent(source.id(), event));
      }
    } while (available.length == 256);
  }
  public function fault():Null<RobotFault> return source.fault();

  public function submit(command:RobotCommand):Void {
    source.submit(command);
    record(function() recording.recordCommand(command, source.id()));
  }

  public function stop(mode:StopMode):Void source.stop(mode);
  public function resetSafety():Void source.resetSafety();

  public function setChangeListener(listener:Null<RobotId->Void>):Void
    source.setChangeListener(listener);

  public function close():Void {
    streamRecording.cancel();
    source.setChangeListener(null);
    source.close();
  }

  function record(action:Void->Void):Void {
    try action() catch (error:Dynamic) {
      if (recordingError == null) recordingError = Std.string(error);
    }
  }
}

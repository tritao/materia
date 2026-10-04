package robotkit.recording;

import robotkit.core.ExecutionCapabilities;
import robotkit.core.JointTargetMode;
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
import robotkit.core.TimingCapabilities;

/**
 * Robot adapter that deterministically replays one recorded robot.
 *
 * ReplayRobot observes commands but does not execute them. Its default
 * capabilities intentionally do not claim queue or plan support. A caller
 * replaying a plan-based controller must pass capabilities with both
 * execution.plans enabled.
 */
class ReplayRobot implements Robot {
  public final logicalId:RobotId;
  /** Commands produced while replaying; never written into the source recording. */
  public final generatedCommands:RobotRecording;
  final descriptionValue:RobotDescription;
  final capabilitiesValue:RobotCapabilities;
  final source:Array<ReplayObservation> = [];
  final eventRing = new RobotEventRing();
  final sensorStreams = new robotkit.streams.SensorStreams();
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
    if (source.length > 0) {
      sensorStreams.publishFrames(source[0].frames);
      for (event in source[0].events) restoreStreamEvent(event);
    }
    descriptionValue = description == null ? new RobotDescription(id, id, [], []) : description;
    capabilitiesValue = capabilities == null
      ? new RobotCapabilities(id,
      source.length == 0 ? 0 : source[0].snapshot.positions.length,
      [robotkit.core.JointTargetMode.Position],
      robotkit.core.ExecutionCapabilities.unavailable(),
      new robotkit.core.TimingCapabilities(false, false, trajectorykit.validation.ValidationGuarantee.Unchecked)) : capabilities;
  }

  public function id():RobotId return logicalId;
  public function status():RobotStatus return closed ? Disconnected : Ready;
  public function description():RobotDescription return descriptionValue;
  public function capabilities():RobotCapabilities return capabilitiesValue;

  public function snapshot():RobotSnapshot {
    ensureLive();
    return source.length == 0 ? emptySnapshot() : RobotRecording.copyRobotSnapshot(source[index].snapshot);
  }

  public function streams():robotkit.streams.SensorStreams {
    ensureLive();
    if (source.length > 0) sensorStreams.publishFrames(source[index].frames);
    return sensorStreams;
  }
  public function events(afterOrdinal:haxe.Int64, max:Int):Array<RobotEvent>
    return eventRing.events(afterOrdinal, max);

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
    sensorStreams.publishFrames(source[index].frames);
    for (event in source[index].events) restoreStreamEvent(event);
    var value = listener;
    if (value != null) value(logicalId);
    return true;
  }

  public function setChangeListener(value:Null<RobotId->Void>):Void listener = value;
  public function close():Void { closed = true; listener = null; }

  function restoreStreamEvent(event:RobotEvent):Void {
    eventRing.restore(event);
    switch event {
      case Observation(_, value): sensorStreams.publish(robotkit.streams.SensorStreamSample.inference(value));
      case Overflow(_, _):
    }
  }

  function buildTimeline(recording:RobotRecording):Void {
    var nextSnapshots:Map<String, RobotSnapshot> = [];
    var nextSnapshot:Null<RobotSnapshot> = null;
    for (offset in 0...recording.entries.length) {
      var entry = recording.entries[recording.entries.length - offset - 1];
      if (nextSnapshot != null) nextSnapshots.set(haxe.Int64.toStr(entry.ordinal), nextSnapshot);
      switch entry.event {
        case RobotSnapshot(value) if (value.id == logicalId): nextSnapshot = value;
        case World(value):
          var selected = value.robot(logicalId);
          if (selected != null) nextSnapshot = selected;
        case _:
      }
    }
    var frames:Array<SensorFrame> = [];
    var current = emptySnapshot();
    var hasSnapshot = false;
    var currentFault:Null<RobotFault> = null;
    for (entry in recording.entries) switch entry.event {
      case RobotSnapshot(value) if (value.id == logicalId):
        hasSnapshot = true;
        current = RobotRecording.copyRobotSnapshot(value);
        frames = mergeFrames(frames, value.sensors.toArray());
        if (value.faultCode == 0) currentFault = null;
        source.push(new ReplayObservation(current, currentFault, null, frames));
      case Sensor(robotId, value) if (robotId == logicalId):
        var sensors = frames.copy();
        var replaced = false;
        for (sensorIndex in 0...sensors.length) if (sensors[sensorIndex].sensorId == value.sensorId) {
          sensors[sensorIndex] = value.copy(); replaced = true; break;
        }
        if (!replaced) sensors.push(value.copy());
        frames = sensors;
        var upcoming = nextSnapshots.get(haxe.Int64.toStr(entry.ordinal));
        var belongsToNext = false;
        if (upcoming != null) for (sequence in upcoming.streamSequences)
          if (sequence.streamId == value.sensorId && sequence.sourceClockId == value.sourceClockId &&
              haxe.Int64.compare(sequence.sequence, value.sequence) == 0) belongsToNext = true;
        if (hasSnapshot && !belongsToNext && source.length > 0 &&
            (value.sourceClockId != current.sourceClockId ||
             haxe.Int64.compare(value.sourceTimestampNs, current.sourceTimestampNs) <= 0)) {
          // Bulk delivery is independent of control observation boundaries.
          var previous = source[source.length - 1];
          source[source.length - 1] = new ReplayObservation(previous.snapshot,
            previous.fault, previous.events, frames);
        }
      case Fault(value) if (value.id == logicalId):
        currentFault = new RobotFault(value.id, value.code, value.message, value.fatal);
        current = copyWith(current, current.sensors.toArray(), value.code);
        source.push(new ReplayObservation(current, currentFault, null, frames));
      case World(value):
        var selected = value.robot(logicalId);
        if (selected != null) {
          hasSnapshot = true;
          current = RobotRecording.copyRobotSnapshot(selected);
          frames = mergeFrames(frames, selected.sensors.toArray());
          if (current.faultCode == 0) currentFault = null;
          source.push(new ReplayObservation(current, currentFault, null, frames));
        }
      case Channel(robotId, "perception.image_detections", payload) if (robotId == logicalId):
        var event:RobotEvent = cast payload;
        source.push(new ReplayObservation(current, currentFault, [event], frames));
      case Command(_), WorldEvent(_), RobotSnapshot(_), Sensor(_, _), Fault(_), ProcessEvent(_), Channel(_, _, _):
    }
    if (!hasSnapshot && frames.length > 0)
      source.push(new ReplayObservation(copyWith(current, frames, current.faultCode), currentFault, null, frames));
  }

  function emptySnapshot():RobotSnapshot return new RobotSnapshot(logicalId,
    haxe.Int64.ofInt(0), haxe.Int64.ofInt(0), [], [], [], 0, 0);

  static function copyWith(value:RobotSnapshot, sensors:Array<SensorFrame>, faultCode:Int):RobotSnapshot
    return new RobotSnapshot(value.id, value.sourceSequence, value.sourceTimestampNs,
      value.positions.toArray(), value.velocities.toArray(), value.efforts.toArray(),
      value.mode, faultCode, value.receivedTimestampNs, sensors,
      value.sourceClockId, value.receivedClockId, value.safety,
      value.trajectoryQueueDepth, value.trajectoryActive,
      value.trajectoryTimeNs, value.trajectoryDurationNs,
      value.trajectoryTag, value.trajectoryTagTimeNs,
      value.sessionState, value.activePlanId,
      value.committedUntilNs, value.queueEndTimeNs, value.setpointPositions.toArray(), mergeSequences(value.streamSequences, sensors));

  static function mergeFrames(previous:Array<SensorFrame>, updates:Array<SensorFrame>):Array<SensorFrame> {
    var frames = [for (frame in previous) frame.copy()];
    for (update in updates) {
      var replaced = false;
      for (index in 0...frames.length) if (frames[index].sensorId == update.sensorId) {
        frames[index] = update.copy(); replaced = true; break;
      }
      if (!replaced) frames.push(update.copy());
    }
    return frames;
  }

  static function mergeSequences(previous:Array<robotkit.streams.StreamSequence>, frames:Array<SensorFrame>):Array<robotkit.streams.StreamSequence> {
    var sequences = previous.copy();
    for (frame in frames) {
      var value = new robotkit.streams.StreamSequence(frame.sensorId, frame.kind, frame.sequence, frame.sourceClockId);
      var replaced = false;
      for (index in 0...sequences.length) if (sequences[index].streamId == frame.sensorId) {
        sequences[index] = value; replaced = true; break;
      }
      if (!replaced) sequences.push(value);
    }
    return sequences;
  }

  function ensureLive():Void if (closed) throw "ReplayRobot has been closed";
}

private class ReplayObservation {
  public final snapshot:RobotSnapshot;
  public final fault:Null<RobotFault>;
  public final events:Array<RobotEvent>;
  public final frames:Array<SensorFrame>;
  public function new(snapshot:RobotSnapshot, fault:Null<RobotFault>, ?events:Array<RobotEvent>, ?frames:Array<SensorFrame>) {
    this.snapshot = RobotRecording.copyRobotSnapshot(snapshot);
    this.fault = fault == null ? null : new RobotFault(fault.id, fault.code, fault.message, fault.fatal);
    this.events = events == null ? [] : events.copy();
    this.frames = frames == null ? snapshot.sensors.toArray() : [for (frame in frames) frame.copy()];
  }
}

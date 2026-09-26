package robotkit.world;

import haxe.Int64;

/** Versioned payload envelope with a recording-wide deterministic ordinal. */
class RobotRecordingEntry {
  public static inline final VERSION:Int = 3;
  public final ordinal:Int64;
  public final robotId:RobotId;
  public final event:RobotRecordingEvent;
  public final sourceSequence:Int64;
  public final sourceTimestampNs:Int64;
  public final sourceClockId:String;
  public final recordingTimestampNs:Int64;
  /** Payload schema used when this entry was loaded, or the current schema for new entries. */
  public final schemaVersion:Int;

  public function new(ordinal:Int64, robotId:RobotId, event:RobotRecordingEvent,
      ?sourceSequence:Int64, ?sourceTimestampNs:Int64, ?sourceClockId:String = "unspecified",
      ?recordingTimestampNs:Int64, ?schemaVersion:Int = 3) {
    this.ordinal = ordinal;
    this.robotId = robotId;
    this.event = event;
    this.sourceSequence = sourceSequence == null ? Int64.ofInt(0) : sourceSequence;
    this.sourceTimestampNs = sourceTimestampNs == null ? Int64.ofInt(0) : sourceTimestampNs;
    this.sourceClockId = sourceClockId;
    this.recordingTimestampNs = recordingTimestampNs == null
      ? Int64.fromFloat(Sys.time() * 1000000000.0) : recordingTimestampNs;
    this.schemaVersion = schemaVersion;
  }
}

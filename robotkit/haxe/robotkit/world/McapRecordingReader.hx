package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;

/** Incremental v6 MCAP cursor. Unknown channels can be skipped or rejected. */
class McapRecordingReader {
  final owner:Ownedrk_recording_reader_handle;
  public final channels:RecordingChannels;
  public final strict:Bool;
  public var skippedUnknown(default, null):Int = 0;
  var previous:Int64 = Int64.ofInt(0);
  var hasPrevious:Bool = false;
  var closed:Bool = false;

  public function new(path:String, ?channels:RecordingChannels, ?strict:Bool = false) {
    this.channels = channels == null ? new RecordingChannels() : channels;
    this.strict = strict;
    var opened = RobotKitRuntime.rk_recording_reader_open(path);
    if (opened.status == RobotKitRuntimeConstants.RK_ERROR_UNSUPPORTED)
      throw "Recording schema version is not 6";
    if (opened.status != RobotKitRuntimeConstants.RK_OK)
      throw 'Open recording failed with RobotKit status ${opened.status}';
    owner = opened.out_reader;
  }

  public function next():Null<RobotRecordingEntry> {
    if (closed) throw "Recording reader is closed";
    while (true) {
      var message = new rk_recording_message();
      message.set_struct_size(rk_recording_message.size());
      var result = RobotKitRuntime.rk_recording_reader_next(owner.borrow(), message);
      if (result.status == RobotKitRuntimeConstants.RK_ERROR_STALE_STATE) return null;
      if (result.status != RobotKitRuntimeConstants.RK_OK)
        throw 'Read recording failed with RobotKit status ${result.status}';
      if (message.get_schema_version() != RobotRecordingEntry.VERSION)
        throw "Recording schema version is not 6";
      if (hasPrevious && Int64.compare(message.get_ordinal(), previous) <= 0)
        throw "Recording ordinals are not strictly increasing";
      previous = message.get_ordinal();
      hasPrevious = true;
      var name = topicName(message);
      var channel = channels.get(name);
      if (channel == null) {
        if (strict) throw 'Unknown recording channel $name';
        skippedUnknown++;
        continue;
      }
      var schema = RobotKitRuntime.rk_recording_reader_schema(owner.borrow());
      if (schema.status != RobotKitRuntimeConstants.RK_OK)
        throw 'Read recording schema failed with RobotKit status ${schema.status}';
      var encoded = schema.schema.toString();
      var separator = encoded.indexOf("\n");
      var data = separator < 0 ? "" : encoded.substr(separator + 1);
      var compatible = data == channel.schemaData ||
        (name == "command" && data == RecordingSchemas.legacyCommand());
      if (separator < 0 || encoded.substr(0, separator) != channel.wireClass || !compatible) {
        if (strict) throw 'Recording schema mismatch for channel $name';
        skippedUnknown++;
        continue;
      }
      var decoded = channel.decode(result.payload);
      var sourceSequence = Int64.ofInt(0);
      var sourceTimestampNs = Int64.ofInt(0);
      var sourceClockId = "unspecified";
      switch decoded.event {
        case RobotSnapshot(value):
          sourceSequence = value.sourceSequence;
          sourceTimestampNs = value.sourceTimestampNs;
          sourceClockId = value.sourceClockId;
        case Sensor(_, value):
          sourceSequence = value.sequence;
          sourceTimestampNs = value.sourceTimestampNs;
          sourceClockId = value.sourceClockId;
        case ProcessEvent(value):
          sourceTimestampNs = value.appliedOwnerTimeNs;
          sourceClockId = "runtime-owner";
        case _:
      }
      return new RobotRecordingEntry(message.get_ordinal(), decoded.robotId,
        decoded.event, sourceSequence, sourceTimestampNs, sourceClockId,
        message.get_recording_timestamp_ns(), RobotRecordingEntry.VERSION);
    }
  }

  static function topicName(message:rk_recording_message):String {
    var length = 0;
    for (index in 0...128) {
      var code = message.get_topic(index);
      if (code == 0) break;
      length++;
    }
    var bytes = haxe.io.Bytes.alloc(length);
    for (index in 0...length) bytes.set(index, message.get_topic(index));
    var topic = bytes.toString();
    if (topic.indexOf("robotkit/") != 0 || topic.length <= 9)
      return topic;
    return topic.substr(9);
  }

  public function close():Void { if (!closed) { owner.close(); closed = true; } }

  public static function load(path:String, ?channels:RecordingChannels,
      ?strict:Bool = false):RobotRecording {
    var reader = new McapRecordingReader(path, channels, strict);
    var recording = new RobotRecording();
    try {
      while (true) {
        var entry = reader.next();
        if (entry == null) break;
        recording.ingest(entry);
      }
    } catch (error:Dynamic) { reader.close(); throw error; }
    reader.close();
    return recording;
  }

  /** Terminal writer status survives process restart in a companion status record. */
  public static function status(path:String):Null<McapRecordingStatus> {
    var statusPath=path+".incomplete.status";
    if(!sys.FileSystem.exists(statusPath))return null;
    var fields=new Map<String,String>();
    for(line in sys.io.File.getContent(statusPath).split("\n")){
      var separator=line.indexOf("=");
      if(separator>0)fields.set(line.substr(0,separator),line.substr(separator+1));
    }
    return new McapRecordingStatus(parseInt(fields,"state"),parseWide(fields,"accepted"),
      parseWide(fields,"written"),parseWide(fields,"dropped"),parseWide(fields,"queued"),
      parseWide(fields,"queued_bytes"),fields.exists("error")?fields.get("error"):"");
  }
  static function parseWide(fields:Map<String,String>,name:String):Int64
    return fields.exists(name)?Int64.parseString(fields.get(name)):Int64.ofInt(0);
  static function parseInt(fields:Map<String,String>,name:String):Int {
    var value=fields.exists(name)?Std.parseInt(fields.get(name)):null;
    return value==null?0:value;
  }
}

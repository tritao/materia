package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;

/** Incremental validated MCAP cursor; only `load` retains a complete recording. */
class McapRecordingReader {
  final owner:Ownedrk_recording_reader_handle;
  var previous:Int64 = Int64.ofInt(0);
  var hasPrevious:Bool = false;
  var closed:Bool = false;

  public function new(path:String) {
    var opened=RobotKitRuntime.rk_recording_reader_open(path);
    if(opened.status!=RobotKitRuntimeConstants.RK_OK)
      throw 'Open recording failed with RobotKit status ${opened.status}';
    owner=opened.out_reader;
  }

  public function next():Null<RobotRecordingEntry> {
    if (closed) throw "Recording reader is closed";
    var message=new rk_recording_message();message.set_struct_size(rk_recording_message.size());
    var result=RobotKitRuntime.rk_recording_reader_next(owner.borrow(),message);
    if(result.status==RobotKitRuntimeConstants.RK_ERROR_STALE_STATE) return null;
    if(result.status!=RobotKitRuntimeConstants.RK_OK)
      throw 'Read recording failed with RobotKit status ${result.status}';
    var entry=RobotRecordingCodec.decode(result.payload);
    if(Int64.compare(entry.ordinal,message.get_ordinal()) != 0)
      throw "Recording ordinal does not match its MCAP envelope";
    if(Int64.compare(entry.recordingTimestampNs,message.get_recording_timestamp_ns()) != 0)
      throw "Recording timestamp does not match its MCAP envelope";
    if(kind(entry.event)!=message.get_kind())
      throw "Recording payload type does not match its MCAP channel";
    if(hasPrevious && Int64.compare(entry.ordinal,previous) <= 0)
      throw "Recording ordinals are not strictly increasing";
    previous=entry.ordinal;hasPrevious=true;return entry;
  }

  public function close():Void { if(!closed){owner.close();closed=true;} }

  public static function load(path:String):RobotRecording {
    var reader=new McapRecordingReader(path),recording=new RobotRecording();
    try { while(true){var entry=reader.next();if(entry==null)break;recording.ingest(cast entry);} }
    catch(error:Dynamic){reader.close();throw error;}
    reader.close();return recording;
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

  static function kind(event:RobotRecordingEvent):Int return switch event {
    case Command(_):1;case SceneSnapshot(_):2;case Sensor(_,_):3;
    case Fault(_):4;case World(_):5;case WorldEvent(_):6;
  };
  static function parseWide(fields:Map<String,String>,name:String):Int64
    return fields.exists(name)?Int64.parseString(fields.get(name)):Int64.ofInt(0);
  static function parseInt(fields:Map<String,String>,name:String):Int {
    var value=fields.exists(name)?Std.parseInt(fields.get(name)):null;
    return value==null?0:value;
  }
}

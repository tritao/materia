package robotkit.world;

import RobotKitRuntime;
import haxe.io.Bytes;

/** Loads validated MCAP observations in recording ordinal order. */
class McapRecordingReader {
  public static function load(path:String):RobotRecording {
    var opened=RobotKitRuntime.rk_recording_reader_open(path);
    if(opened.status!=RobotKitRuntimeConstants.RK_OK) throw 'Open recording failed with RobotKit status ${opened.status}';
    var owner=opened.out_reader, recording=new RobotRecording();
    var previous=haxe.Int64.ofInt(0), hasPrevious=false;
    try {
      while(true) {
        var message=new rk_recording_message();message.set_struct_size(rk_recording_message.size());
        var result=RobotKitRuntime.rk_recording_reader_next(owner.borrow(),message);
        if(result.status==RobotKitRuntimeConstants.RK_ERROR_STALE_STATE) break;
        if(result.status!=RobotKitRuntimeConstants.RK_OK) throw 'Read recording failed with RobotKit status ${result.status}';
        var entry=RobotRecordingCodec.decode(result.payload);
        if(haxe.Int64.compare(entry.ordinal,message.get_ordinal()) != 0) throw "Recording ordinal does not match its MCAP envelope";
        if(hasPrevious && haxe.Int64.compare(entry.ordinal,previous) <= 0) throw "Recording ordinals are not strictly increasing";
        previous=entry.ordinal; hasPrevious=true; recording.ingest(entry);
      }
    } catch(error:Dynamic) { owner.close(); throw error; }
    owner.close(); return recording;
  }
}

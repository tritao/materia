package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;

/** File-backed recording facade. MCAP types and threading stay below this boundary. */
class McapRobotRecording {
  public final memory:RobotRecording;
  final owner:Ownedrk_recording_writer_handle;
  var closed:Bool = false;

  public function new(path:String, ?queueCapacity:Int = 1024) {
    if (queueCapacity <= 0) throw "Recording queue capacity must be positive";
    var result = RobotKitRuntime.rk_recording_writer_create(path, queueCapacity);
    check(result.status, "open recording");
    owner = result.out_writer;
    memory = new RobotRecording();
  }

  public function recordCommand(command:RobotCommand, ?robotId:RobotId = ""):Void {
    memory.recordCommand(command, robotId); flushLast();
  }
  public function recordSnapshot(value:RobotSnapshot):Void { memory.recordSnapshot(value); flushLast(); }
  public function recordSensor(robotId:RobotId, value:SensorFrame):Void { memory.recordSensor(robotId,value); flushLast(); }
  public function recordFault(value:RobotFault):Void { memory.recordFault(value); flushLast(); }
  public function recordWorld(value:WorldSnapshot):Void { memory.recordWorld(value); flushLast(); }
  public function recordEvent(value:RobotWorldEvent):Void { memory.recordEvent(value); flushLast(); }
  public function attach(world:RobotWorld):RobotWorldSubscription return world.subscribe(recordEvent);

  public function status():McapRecordingStatus {
    var value = new rk_recording_writer_status(); value.set_struct_size(rk_recording_writer_status.size());
    check(RobotKitRuntime.rk_recording_writer_get_status(owner.borrow(), value), "read recording status");
    var error = new StringBuf();
    for (index in 0...256) { var code=value.get_error(index); if(code==0) break; error.addChar(code); }
    return new McapRecordingStatus(value.get_state(),value.get_accepted(),value.get_written(),value.get_dropped(),value.get_queued(),error.toString());
  }

  /** Drains the queue and writes the MCAP footer. Failure is always surfaced. */
  public function close():Void {
    if (closed) return;
    var result = RobotKitRuntime.rk_recording_writer_finish(owner.borrow());
    var finalStatus = status();
    owner.close(); closed=true;
    if (result != RobotKitRuntimeConstants.RK_OK || Int64.compare(finalStatus.dropped,Int64.ofInt(0)) != 0 || finalStatus.error.length != 0)
      throw 'Recording did not finish cleanly: dropped=${Int64.toStr(finalStatus.dropped)} ${finalStatus.error}';
  }

  function flushLast():Void {
    if (closed) throw "Recording is closed";
    var entry=memory.entries[memory.entries.length-1], bytes=RobotRecordingCodec.encode(entry);
    var kind = switch entry.event {case Command(_):1;case Snapshot(_):2;case Sensor(_,_):3;case Fault(_):4;case World(_):5;case WorldEvent(_):6;};
    check(RobotKitRuntime.rk_recording_writer_enqueue(owner.borrow(),kind,RobotRecordingEntry.VERSION,entry.ordinal,bytes), "enqueue recording event");
  }
  static function check(status:Int, operation:String):Void if(status!=RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
}

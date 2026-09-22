package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;

/** File-backed recording facade. MCAP types and threading stay below this boundary. */
class McapRobotRecording {
  /** Null when file-only recording was requested. */
  public final memory:Null<RobotRecording>;
  final staging:RobotRecording;
  final retainInMemory:Bool;
  final owner:Ownedrk_recording_writer_handle;
  var closed:Bool = false;

  public function new(path:String, ?queueCapacityBytes:Int = 16 * 1024 * 1024,
      ?retainInMemory:Bool = true) {
    if (queueCapacityBytes <= 0) throw "Recording queue byte capacity must be positive";
    var result = RobotKitRuntime.rk_recording_writer_create(path, Int64.ofInt(queueCapacityBytes));
    check(result.status, "open recording");
    owner = result.out_writer;
    this.retainInMemory = retainInMemory;
    staging = new RobotRecording();
    memory = retainInMemory ? staging : null;
  }

  public function recordCommand(command:RobotCommand, ?robotId:RobotId = ""):Void {
    staging.recordCommand(command, robotId); flushLast();
  }
  public function recordSnapshot(value:RobotSnapshot):Void { staging.recordSnapshot(value); flushLast(); }
  public function recordSensor(robotId:RobotId, value:SensorFrame):Void { staging.recordSensor(robotId,value); flushLast(); }
  public function recordFault(value:RobotFault):Void { staging.recordFault(value); flushLast(); }
  public function recordWorld(value:WorldSnapshot):Void { staging.recordWorld(value); flushLast(); }
  public function recordEvent(value:RobotWorldEvent):Void { staging.recordEvent(value); flushLast(); }
  public function attach(world:RobotWorld):RobotWorldSubscription return world.subscribe(recordEvent);

  public function status():McapRecordingStatus {
    var value = new rk_recording_writer_status(); value.set_struct_size(rk_recording_writer_status.size());
    check(RobotKitRuntime.rk_recording_writer_get_status(owner.borrow(), value), "read recording status");
    var error = new StringBuf();
    for (index in 0...256) { var code=value.get_error(index); if(code==0) break; error.addChar(code); }
    return new McapRecordingStatus(value.get_state(),value.get_accepted(),value.get_written(),
      value.get_dropped(),value.get_queued(),value.get_queued_bytes(),error.toString());
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
    var entry=staging.entries[staging.entries.length-1], bytes=RobotRecordingCodec.encode(entry);
    var kind = switch entry.event {case Command(_):1;case Snapshot(_):2;case Sensor(_,_):3;case Fault(_):4;case World(_):5;case WorldEvent(_):6;};
    check(RobotKitRuntime.rk_recording_writer_enqueue(owner.borrow(),kind,
      RobotRecordingEntry.VERSION,entry.ordinal,entry.recordingTimestampNs,bytes),
      "enqueue recording event");
    if (!retainInMemory) clearStaging();
  }
  function clearStaging():Void {
    staging.commands.splice(0, staging.commands.length);
    staging.snapshots.splice(0, staging.snapshots.length);
    staging.faults.splice(0, staging.faults.length);
    staging.worlds.splice(0, staging.worlds.length);
    staging.events.splice(0, staging.events.length);
    staging.entries.splice(0, staging.entries.length);
  }
  static function check(status:Int, operation:String):Void if(status!=RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
}

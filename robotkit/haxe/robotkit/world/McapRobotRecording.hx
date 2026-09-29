package robotkit.world;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.world.RecordingChannels.RegisteredChannel;

/** File-backed recording facade. MCAP types and threading stay below this boundary. */
class McapRobotRecording implements RobotRecordingSink {
  /** Null when file-only recording was requested. */
  public final memory:Null<RobotRecording>;
  final staging:RobotRecording;
  final retainInMemory:Bool;
  final owner:Ownedrk_recording_writer_handle;
  public final channels:RecordingChannels;
  final channelIds:Map<String, Int> = new Map<String, Int>();
  var closed:Bool = false;

  public function new(path:String, ?queueCapacityBytes:Int = 16 * 1024 * 1024,
      ?retainInMemory:Bool = true, ?compression:String = "lz4",
      ?channels:RecordingChannels) {
    if (queueCapacityBytes <= 0) throw "Recording queue byte capacity must be positive";
    var compressionValue = switch compression {
      case "none": RobotKitRuntimeConstants.RK_RECORDING_COMPRESSION_NONE;
      case "lz4": RobotKitRuntimeConstants.RK_RECORDING_COMPRESSION_LZ4;
      case _: throw "Recording compression must be none or lz4";
    };
    var result = RobotKitRuntime.rk_recording_writer_create(path,
      Int64.ofInt(queueCapacityBytes), compressionValue);
    check(result.status, "open recording");
    owner = result.out_writer;
    this.channels = channels == null ? new RecordingChannels() : channels;
    this.retainInMemory = retainInMemory;
    staging = new RobotRecording();
    memory = retainInMemory ? staging : null;
    try {
      for (channel in this.channels.channels()) registerChannel(channel);
    } catch (error:Dynamic) {
      owner.close();
      throw error;
    }
  }

  public function recordCommand(command:RobotCommand, ?robotId:RobotId = ""):Void {
    staging.recordCommand(command, robotId); flushLast();
  }
  public function recordSnapshot(value:RobotSnapshot):Void { staging.recordSnapshot(value); flushLast(); }
  public function recordSensor(robotId:RobotId, value:SensorFrame):Void { staging.recordSensor(robotId,value); flushLast(); }
  public function recordFault(value:RobotFault):Void { staging.recordFault(value); flushLast(); }
  public function recordWorld(value:WorldSnapshot):Void { staging.recordWorld(value); flushLast(); }
  public function recordEvent(value:RobotWorldEvent):Void { staging.recordEvent(value); flushLast(); }
  public function recordProcessEvent(robotId:RobotId, value:FiredProcessEvent):Void {
    staging.recordProcessEvent(robotId, value); flushLast();
  }
  public function recordChannel(robotId:RobotId, name:String, payload:Dynamic):Void {
    if (channels.get(name) == null) throw 'Unregistered recording channel $name';
    staging.recordChannel(robotId, name, payload); flushLast();
  }
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
    var entry=staging.entries[staging.entries.length-1];
    var name = RecordingChannels.nameOf(entry.event);
    var channel = channels.get(name);
    if (channel == null) throw 'Unregistered recording channel $name';
    if (!channelIds.exists(name)) registerChannel(channel);
    var channelId = channelIds.get(name);
    if (channelId == null) throw 'Recording channel $name has no native ID';
    var bytes = channel.encode(entry);
    check(RobotKitRuntime.rk_recording_writer_enqueue(owner.borrow(),channelId,
      entry.ordinal,entry.recordingTimestampNs,bytes),
      "enqueue recording event");
    if (!retainInMemory) clearStaging();
  }
  function registerChannel(channel:RegisteredChannel):Void {
    var result = RobotKitRuntime.rk_recording_writer_register_channel(owner.borrow(),
      "robotkit/" + channel.name, channel.wireClass, "robotkit-wire",
      haxe.io.Bytes.ofString(channel.schemaData), "msgpack");
    check(result.status, "register recording channel");
    channelIds.set(channel.name, result.out_channel_id);
  }
  function clearStaging():Void {
    staging.commands.splice(0, staging.commands.length);
    staging.snapshots.splice(0, staging.snapshots.length);
    staging.faults.splice(0, staging.faults.length);
    staging.worlds.splice(0, staging.worlds.length);
    staging.processEvents.splice(0, staging.processEvents.length);
    staging.events.splice(0, staging.events.length);
    staging.entries.splice(0, staging.entries.length);
  }
  static function check(status:Int, operation:String):Void if(status!=RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
}

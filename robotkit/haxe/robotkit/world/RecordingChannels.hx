package robotkit.world;

import haxe.io.Bytes;
import robotkit.world.CoreRecordingChannels.CoreCommandChannel;
import robotkit.world.CoreRecordingChannels.CoreSnapshotChannel;
import robotkit.world.CoreRecordingChannels.CoreSensorChannel;
import robotkit.world.CoreRecordingChannels.CoreFaultChannel;
import robotkit.world.CoreRecordingChannels.CoreWorldChannel;
import robotkit.world.CoreRecordingChannels.CoreWorldEventChannel;
import robotkit.world.CoreRecordingChannels.CoreProcessEventChannel;

class DecodedRecordingPayload {
  public final robotId:RobotId;
  public final event:RobotRecordingEvent;
  public function new(robotId:RobotId, event:RobotRecordingEvent) {
    this.robotId = robotId;
    this.event = event;
  }
}

class RegisteredChannel {
  public final name:String;
  public final wireClass:String;
  public final schemaData:String;
  final encoder:RobotRecordingEntry->Bytes;
  final decoder:Bytes->DecodedRecordingPayload;
  public function new(name:String, wireClass:String, schemaData:String,
      encoder:RobotRecordingEntry->Bytes, decoder:Bytes->DecodedRecordingPayload) {
    this.name = name;
    this.wireClass = wireClass;
    this.schemaData = schemaData;
    this.encoder = encoder;
    this.decoder = decoder;
  }
  public function encode(entry:RobotRecordingEntry):Bytes return encoder(entry);
  public function decode(bytes:Bytes):DecodedRecordingPayload return decoder(bytes);
}

/** Heterogeneous registry with a typed codec at each registration site. */
class RecordingChannels {
  final byName:Map<String, RegisteredChannel> = new Map<String, RegisteredChannel>();
  final ordered:Array<RegisteredChannel> = [];

  public function new() {
    register(new CoreCommandChannel());
    register(new CoreSnapshotChannel());
    register(new CoreSensorChannel());
    register(new CoreFaultChannel());
    register(new CoreWorldChannel());
    register(new CoreWorldEventChannel());
    register(new CoreProcessEventChannel());
    register(new PerceptionRecordingChannel());
  }

  public function register<T>(channel:RecordingChannel<T>):Void {
    if (channel == null || channel.name() == null || channel.name().length == 0 ||
        channel.wireClass() == null || channel.wireClass().length == 0 ||
        channel.schemaData() == null || channel.schemaData().length == 0 ||
        byName.exists(channel.name()))
      throw "Invalid or duplicate recording channel";
    var entry = new RegisteredChannel(channel.name(), channel.wireClass(), channel.schemaData(),
      function(value) return channel.encode(channel.toWire(value)),
      function(bytes) {
        var value = channel.decode(bytes);
        return new DecodedRecordingPayload(channel.robotId(value), channel.fromWire(value));
      });
    byName.set(entry.name, entry);
    ordered.push(entry);
  }

  public function get(name:String):Null<RegisteredChannel> return byName.get(name);
  public function channels():Array<RegisteredChannel> return ordered.copy();

  public static function nameOf(event:RobotRecordingEvent):String return switch event {
    case Command(_): "command";
    case RobotSnapshot(_): "snapshot";
    case Sensor(_, _): "sensor";
    case Fault(_): "fault";
    case World(_): "world";
    case WorldEvent(_): "world_event";
    case ProcessEvent(_): "process_event";
    case Channel(_, name, _): name;
  };
}

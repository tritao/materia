package robotkit.world;

import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import robotkit.protocol.RecordingCommandMsg;
import robotkit.protocol.RecordingSnapshotMsg;
import robotkit.protocol.RecordingSensorMsg;
import robotkit.protocol.RecordingFaultMsg;
import robotkit.protocol.RecordingWorldMsg;
import robotkit.protocol.RecordingWorldEventMsg;
import robotkit.protocol.RecordingProcessEventMsg;

/** Core channels use the same registration interface as external channels. */
class CoreCommandChannel implements RecordingChannel<RecordingCommandMsg> {
  public function new() {}
  public function name():String return "command";
  public function wireClass():String return "RecordingCommandMsg";
  public function schemaData():String return RecordingSchemas.forChannel("command");
  public function toWire(entry:RobotRecordingEntry):RecordingCommandMsg return switch entry.event {
    case Command(value): RobotRecordingCodec.command(value, entry.robotId);
    case _: throw "Recording event is not on command channel";
  };
  public function fromWire(value:RecordingCommandMsg):RobotRecordingEvent return Command(RobotRecordingCodec.readCommand(value));
  public function robotId(value:RecordingCommandMsg):RobotId return value.robotId;
  public function encode(value:RecordingCommandMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingCommandMsg return MessagePack.decode(bytes);
}

class CoreSnapshotChannel implements RecordingChannel<RecordingSnapshotMsg> {
  public function new() {}
  public function name():String return "snapshot";
  public function wireClass():String return "RecordingSnapshotMsg";
  public function schemaData():String return RecordingSchemas.forChannel("snapshot");
  public function toWire(entry:RobotRecordingEntry):RecordingSnapshotMsg return switch entry.event {
    case RobotSnapshot(value): RobotRecordingCodec.snapshot(value);
    case _: throw "Recording event is not on snapshot channel";
  };
  public function fromWire(value:RecordingSnapshotMsg):RobotRecordingEvent return RobotSnapshot(RobotRecordingCodec.readSnapshot(value));
  public function robotId(value:RecordingSnapshotMsg):RobotId return value.id;
  public function encode(value:RecordingSnapshotMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingSnapshotMsg return MessagePack.decode(bytes);
}

class CoreSensorChannel implements RecordingChannel<RecordingSensorMsg> {
  public function new() {}
  public function name():String return "sensor";
  public function wireClass():String return "RecordingSensorMsg";
  public function schemaData():String return RecordingSchemas.forChannel("sensor");
  public function toWire(entry:RobotRecordingEntry):RecordingSensorMsg return switch entry.event {
    case Sensor(robotId, value): RobotRecordingCodec.sensor(value, robotId);
    case _: throw "Recording event is not on sensor channel";
  };
  public function fromWire(value:RecordingSensorMsg):RobotRecordingEvent return Sensor(value.robotId, RobotRecordingCodec.readSensor(value));
  public function robotId(value:RecordingSensorMsg):RobotId return value.robotId;
  public function encode(value:RecordingSensorMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingSensorMsg return MessagePack.decode(bytes);
}

class CoreFaultChannel implements RecordingChannel<RecordingFaultMsg> {
  public function new() {}
  public function name():String return "fault";
  public function wireClass():String return "RecordingFaultMsg";
  public function schemaData():String return RecordingSchemas.forChannel("fault");
  public function toWire(entry:RobotRecordingEntry):RecordingFaultMsg return switch entry.event {
    case Fault(value): RobotRecordingCodec.fault(value);
    case _: throw "Recording event is not on fault channel";
  };
  public function fromWire(value:RecordingFaultMsg):RobotRecordingEvent return Fault(RobotRecordingCodec.readFault(value));
  public function robotId(value:RecordingFaultMsg):RobotId return value.id;
  public function encode(value:RecordingFaultMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingFaultMsg return MessagePack.decode(bytes);
}

class CoreWorldChannel implements RecordingChannel<RecordingWorldMsg> {
  public function new() {}
  public function name():String return "world";
  public function wireClass():String return "RecordingWorldMsg";
  public function schemaData():String return RecordingSchemas.forChannel("world");
  public function toWire(entry:RobotRecordingEntry):RecordingWorldMsg return switch entry.event {
    case World(value): RobotRecordingCodec.world(value);
    case _: throw "Recording event is not on world channel";
  };
  public function fromWire(value:RecordingWorldMsg):RobotRecordingEvent return World(RobotRecordingCodec.readWorld(value));
  public function robotId(value:RecordingWorldMsg):RobotId return "";
  public function encode(value:RecordingWorldMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingWorldMsg return MessagePack.decode(bytes);
}

class CoreWorldEventChannel implements RecordingChannel<RecordingWorldEventMsg> {
  public function new() {}
  public function name():String return "world_event";
  public function wireClass():String return "RecordingWorldEventMsg";
  public function schemaData():String return RecordingSchemas.forChannel("world_event");
  public function toWire(entry:RobotRecordingEntry):RecordingWorldEventMsg return switch entry.event {
    case WorldEvent(value): RobotRecordingCodec.worldEvent(value);
    case _: throw "Recording event is not on world_event channel";
  };
  public function fromWire(value:RecordingWorldEventMsg):RobotRecordingEvent return WorldEvent(RobotRecordingCodec.readWorldEvent(value));
  public function robotId(value:RecordingWorldEventMsg):RobotId return value.robotId;
  public function encode(value:RecordingWorldEventMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingWorldEventMsg return MessagePack.decode(bytes);
}

class CoreProcessEventChannel implements RecordingChannel<RecordingProcessEventMsg> {
  public function new() {}
  public function name():String return "process_event";
  public function wireClass():String return "RecordingProcessEventMsg";
  public function schemaData():String return RecordingSchemas.forChannel("process_event");
  public function toWire(entry:RobotRecordingEntry):RecordingProcessEventMsg return switch entry.event {
    case ProcessEvent(value): RobotRecordingCodec.processEvent(value, entry.robotId);
    case _: throw "Recording event is not on process_event channel";
  };
  public function fromWire(value:RecordingProcessEventMsg):RobotRecordingEvent return ProcessEvent(RobotRecordingCodec.readProcessEvent(value));
  public function robotId(value:RecordingProcessEventMsg):RobotId return value.robotId;
  public function encode(value:RecordingProcessEventMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):RecordingProcessEventMsg return MessagePack.decode(bytes);
}

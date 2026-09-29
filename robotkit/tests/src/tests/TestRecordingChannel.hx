package tests;

import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import robotkit.world.RecordingChannel;
import robotkit.world.RobotRecordingEntry;
import robotkit.world.RobotRecordingEvent;
import robotkit.world.RobotId;

class TestRecordingChannel implements RecordingChannel<TestRecordingPayload> {
  public function new() {}
  public function name():String return "custom";
  public function wireClass():String return "TestRecordingPayload";
  public function schemaData():String return '{"root":"TestRecordingPayload","declarations":{"TestRecordingPayload":{"kind":"class","fields":[{"id":1,"name":"robotId","type":"String"},{"id":2,"name":"value","type":"Int"}]}}}';
  public function toWire(entry:RobotRecordingEntry):TestRecordingPayload return switch entry.event {
    case Channel(_, "custom", payload): cast payload;
    case _: throw "Not a custom recording event";
  };
  public function fromWire(value:TestRecordingPayload):RobotRecordingEvent return Channel(value.robotId, "custom", value);
  public function robotId(value:TestRecordingPayload):RobotId return value.robotId;
  public function encode(value:TestRecordingPayload):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):TestRecordingPayload return MessagePack.decode(bytes);
}

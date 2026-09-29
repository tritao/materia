package robotkit.world;

import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import robotkit.protocol.ImageDetectionObservationMsg;

/** The same wire value is stored in MCAP and sent over RKF1. */
class PerceptionRecordingChannel implements RecordingChannel<ImageDetectionObservationMsg> {
  public function new() {}
  public function name():String return "perception.image_detections";
  public function wireClass():String return "ImageDetectionObservationMsg";
  public function schemaData():String return RecordingSchemas.forChannel(name());
  public function toWire(entry:RobotRecordingEntry):ImageDetectionObservationMsg return switch entry.event {
    case Channel(_, "perception.image_detections", payload):
      var event:RobotEvent = cast payload;
      switch event {
        case Observation(ordinal, observation):
          var value = ImageDetectionObservationMsg.fromObservation(Int64.ofInt(0), ordinal, observation);
          value.logicalRobotId = entry.robotId;
          value;
        case Overflow(_, _): throw "Overflow is not a detection observation";
      }
    case _: throw "Recording event is not a perception observation";
  };
  public function fromWire(value:ImageDetectionObservationMsg):RobotRecordingEvent
    return Channel(value.logicalRobotId, name(), RobotEvent.Observation(value.ordinal, value.toObservation()));
  public function robotId(value:ImageDetectionObservationMsg):RobotId return value.logicalRobotId;
  public function encode(value:ImageDetectionObservationMsg):Bytes return MessagePack.encode(value);
  public function decode(bytes:Bytes):ImageDetectionObservationMsg return MessagePack.decode(bytes);
}

package robotkit.protocol;

import haxe.Int64;
import robotkit.perception.ImageDetectionObservation;

@:wire
class ImageDetectionObservationMsg {
  @:id(1) public var robotId:Int64;
  @:id(2) public var ordinal:Int64;
  @:id(3) public var producerId:String;
  @:id(4) public var pipelineId:String;
  @:id(5) public var sensorId:String;
  @:id(6) public var modelId:String;
  @:id(7) public var modelDigest:String;
  @:id(8) public var sourceFrameId:String;
  @:id(9) public var sequence:Int64;
  @:id(10) public var sourceTimestampNs:Int64;
  @:id(11) public var receivedTimestampNs:Int64;
  @:id(12) public var sourceClockId:String;
  @:id(13) public var receivedClockId:String;
  @:id(14) public var completedTimestampNs:Int64;
  @:id(15) public var completedClockId:String;
  @:id(16) public var detections:Array<ImageDetectionMsg>;
  @:id(17) public var droppedFrames:Int;
  @:id(18) public var logicalRobotId:String;

  public function new(?robotId:Int64, ?ordinal:Int64, ?producerId:String = "",
      ?pipelineId:String = "", ?sensorId:String = "", ?modelId:String = "",
      ?modelDigest:String = "", ?sourceFrameId:String = "", ?sequence:Int64,
      ?sourceTimestampNs:Int64, ?receivedTimestampNs:Int64,
      ?sourceClockId:String = "", ?receivedClockId:String = "",
      ?completedTimestampNs:Int64, ?completedClockId:String = "",
      ?detections:Array<ImageDetectionMsg>, ?droppedFrames:Int = 0) {
    this.robotId = robotId == null ? Int64.ofInt(0) : robotId;
    this.ordinal = ordinal == null ? Int64.ofInt(0) : ordinal;
    this.producerId = producerId; this.pipelineId = pipelineId;
    this.sensorId = sensorId; this.modelId = modelId; this.modelDigest = modelDigest;
    this.sourceFrameId = sourceFrameId;
    this.sequence = sequence == null ? Int64.ofInt(0) : sequence;
    this.sourceTimestampNs = sourceTimestampNs == null ? Int64.ofInt(0) : sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null ? Int64.ofInt(0) : receivedTimestampNs;
    this.sourceClockId = sourceClockId; this.receivedClockId = receivedClockId;
    this.completedTimestampNs = completedTimestampNs == null ? Int64.ofInt(0) : completedTimestampNs;
    this.completedClockId = completedClockId;
    this.detections = detections == null ? [] : detections.copy();
    this.droppedFrames = droppedFrames;
    this.logicalRobotId = "";
  }

  public static function fromObservation(robotId:Int64, ordinal:Int64,
      value:ImageDetectionObservation):ImageDetectionObservationMsg
    return new ImageDetectionObservationMsg(robotId, ordinal, value.producerId,
      value.pipelineId, value.sensorId, value.modelId, value.modelDigest,
      value.sourceFrameId, value.sequence, value.sourceTimestampNs,
      value.receivedTimestampNs, value.sourceClockId, value.receivedClockId,
      value.completedTimestampNs, value.completedClockId,
      [for (detection in value.detections) ImageDetectionMsg.fromDetection(detection)],
      value.droppedFrames);

  public function toObservation():ImageDetectionObservation
    return new ImageDetectionObservation(producerId, pipelineId, sensorId, modelId,
      modelDigest, sourceFrameId, sequence, sourceTimestampNs, receivedTimestampNs,
      sourceClockId, receivedClockId, completedTimestampNs, completedClockId,
      [for (detection in detections) detection.toDetection()], droppedFrames);
}

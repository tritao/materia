package robotkit.perception;

import haxe.Int64;

/** Immutable model output with the source image's identity and distinct clocks. */
class ImageDetectionObservation {
  public final producerId:String;
  public final pipelineId:String;
  public final sensorId:String;
  public final modelId:String;
  public final modelDigest:String;
  public final sourceFrameId:String;
  public final sequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;
  public final completedTimestampNs:Int64;
  public final completedClockId:String;
  public final droppedFrames:Int;
  final values:Array<ImageDetection>;
  public var detections(get, never):Array<ImageDetection>;

  public function new(producerId:String, pipelineId:String, sensorId:String,
      modelId:String, modelDigest:String, sourceFrameId:String, sequence:Int64,
      sourceTimestampNs:Int64, receivedTimestampNs:Int64, sourceClockId:String,
      receivedClockId:String, completedTimestampNs:Int64, completedClockId:String,
      detections:Array<ImageDetection>, droppedFrames:Int) {
    if (producerId == null || producerId.length == 0 || pipelineId == null || pipelineId.length == 0 ||
        sensorId == null || sensorId.length == 0 || modelId == null || modelId.length == 0 ||
        modelDigest == null || !~/^[0-9a-f]{64}$/.match(modelDigest) ||
        sourceFrameId == null || sourceFrameId.length == 0 ||
        sourceClockId == null || sourceClockId.length == 0 ||
        receivedClockId == null || receivedClockId.length == 0 ||
        completedClockId == null || completedClockId.length == 0 ||
        detections == null || droppedFrames < 0)
      throw "ImageDetectionObservation requires provenance, clocks, and detections";
    for (value in detections) if (value == null) throw "Null image detection";
    this.producerId = producerId; this.pipelineId = pipelineId;
    this.sensorId = sensorId; this.modelId = modelId; this.modelDigest = modelDigest;
    this.sourceFrameId = sourceFrameId; this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId; this.receivedClockId = receivedClockId;
    this.completedTimestampNs = completedTimestampNs; this.completedClockId = completedClockId;
    this.values = detections.copy(); this.droppedFrames = droppedFrames;
  }
  function get_detections():Array<ImageDetection> return values.copy();
}

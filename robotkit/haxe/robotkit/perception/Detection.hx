package robotkit.perception;

import haxe.Int64;
import robotkit.mobile.Pose2;

/** Immutable semantic detection derived from one source observation. */
class Detection {
  public final id:String;
  public final kind:String;
  public final confidence:Float;
  public final pose:Pose2;
  public final frameId:String;
  public final sourceSequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;

  public function new(id:String, kind:String, confidence:Float, pose:Pose2,
      frameId:String, sourceSequence:Int64, sourceTimestampNs:Int64,
      receivedTimestampNs:Int64, sourceClockId:String, receivedClockId:String) {
    if (id == null || id.length == 0 || kind == null || kind.length == 0 ||
        pose == null || frameId == null || frameId.length == 0 ||
        !Math.isFinite(confidence) || confidence < 0.0 || confidence > 1.0 ||
        sourceClockId == null || sourceClockId.length == 0 ||
        receivedClockId == null || receivedClockId.length == 0)
      throw "Detection requires identity, frame, pose, confidence, and clock metadata";
    this.id = id;
    this.kind = kind;
    this.confidence = confidence;
    this.pose = new Pose2(pose.x, pose.y, pose.yaw);
    this.frameId = frameId;
    this.sourceSequence = sourceSequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
  }
}

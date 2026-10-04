package robotkit.localization;

import haxe.Int64;
import robotkit.mobile.Pose2;

/** A pose estimate with explicit frame, uncertainty, and clock identity. */
class LocalizationState {
  public final sequence:Int64;
  /** The pose of `bodyFrame` expressed in `referenceFrame`. */
  public final pose:Pose2;
  public final referenceFrame:String;
  public final bodyFrame:String;
  public final covariance:PoseCovariance2;
  public final quality:LocalizationQuality;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;

  public function new(sequence:Int64, pose:Pose2, referenceFrame:String, bodyFrame:String,
      covariance:PoseCovariance2, quality:LocalizationQuality,
      sourceTimestampNs:Int64, receivedTimestampNs:Int64,
      sourceClockId:String, receivedClockId:String) {
    if (pose == null || covariance == null || quality == null)
      throw "Localization state requires a pose, covariance, and quality";
    if (referenceFrame == null || referenceFrame.length == 0 ||
        bodyFrame == null || bodyFrame.length == 0 || referenceFrame == bodyFrame)
      throw "Localization state requires distinct non-empty frame IDs";
    if (sourceClockId == null || sourceClockId.length == 0 ||
        receivedClockId == null || receivedClockId.length == 0)
      throw "Localization state requires source and receive clock IDs";
    this.sequence = sequence;
    this.pose = pose;
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
    this.covariance = covariance;
    this.quality = quality;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
  }
}

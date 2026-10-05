package robotkit.runtime;

import haxe.Int64;

/** Controller-owned digital release handshake. Physical effort and delays belong to the device. */
class RobotRuntimeBrakeBlueprint {
  public final joint:Int;
  public final releaseChannel:String;
  public final releasedSensor:String;
  public final releaseTimeoutNs:Int64;
  public final feedbackMaxAgeNs:Int64;

  public function new(joint:Int, releaseChannel:String, releasedSensor:String,
      releaseTimeoutNs:Int64, feedbackMaxAgeNs:Int64) {
    this.joint = joint;
    this.releaseChannel = releaseChannel;
    this.releasedSensor = releasedSensor;
    this.releaseTimeoutNs = releaseTimeoutNs;
    this.feedbackMaxAgeNs = feedbackMaxAgeNs;
  }
}

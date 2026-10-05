package robotkit.model;

/** Spring-applied brake on a motor joint. Effort and delays describe the physical device;
 * release timeout and feedback age describe the controller's admission policy. */
class Brake {
  public final id:String;
  public final joint:String;
  public final releaseChannel:String;
  public final releasedSensor:String;
  public final holdingEffort:Float;
  public final releaseDelay:Float;
  public final applicationDelay:Float;
  public final releaseTimeoutMs:Int;
  public final feedbackMaxAgeMs:Int;

  public function new(id:String, joint:String, releaseChannel:String, releasedSensor:String,
      holdingEffort:Float, releaseDelay:Float, applicationDelay:Float,
      releaseTimeoutMs:Int, feedbackMaxAgeMs:Int) {
    if (id == null || id.length == 0 || joint == null || joint.length == 0 ||
        releaseChannel == null || releaseChannel.length == 0 ||
        releasedSensor == null || releasedSensor.length == 0)
      throw "Brake needs a joint, release channel and contact sensor";
    if (!(holdingEffort > 0) || !Math.isFinite(holdingEffort) ||
        releaseDelay < 0 || !Math.isFinite(releaseDelay) ||
        applicationDelay < 0 || !Math.isFinite(applicationDelay))
      throw "Brake needs finite positive effort and nonnegative physical delays";
    if (releaseTimeoutMs <= 0 || feedbackMaxAgeMs <= 0 ||
        releaseTimeoutMs <= releaseDelay * 1000)
      throw "Brake release timeout must exceed its physical release delay";
    this.id = id;
    this.joint = joint;
    this.releaseChannel = releaseChannel;
    this.releasedSensor = releasedSensor;
    this.holdingEffort = holdingEffort;
    this.releaseDelay = releaseDelay;
    this.applicationDelay = applicationDelay;
    this.releaseTimeoutMs = releaseTimeoutMs;
    this.feedbackMaxAgeMs = feedbackMaxAgeMs;
  }
}

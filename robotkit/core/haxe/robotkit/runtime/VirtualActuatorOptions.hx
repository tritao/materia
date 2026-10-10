package robotkit.runtime;

/** One ordered RKD6 channel and its SimpleTransmission into a model joint. */
class VirtualActuatorOptions {
  public final id:String;
  public final jointIndex:Int;
  public final ratio:Float;
  public final offset:Float;
  public final stepsPerUnit:Float;
  /** The least whole step-tick periods between steps (`StepTicks`). */
  public final minStepTicks:Int;
  public final directionSetupTicks:Int;
  public final skewBound:Float;
  public final feedbackJointIndex:Int;
  public final feedbackRatio:Float;
  public final feedbackOffset:Float;

  public function new(id:String, jointIndex:Int, ratio:Float, offset:Float,
      stepsPerUnit:Float, minStepTicks:Int = 1, directionSetupTicks:Int = 0,
      skewBound:Float = 0.0, feedbackJointIndex:Int = -1,
      feedbackRatio:Float = 1.0, feedbackOffset:Float = 0.0) {
    if (id == null || id.length == 0 || id.length > 63 ||
        jointIndex < 0 || jointIndex >= 64 || !Math.isFinite(ratio) || ratio == 0.0 ||
        !Math.isFinite(offset) || !Math.isFinite(stepsPerUnit) || stepsPerUnit <= 0.0 ||
        minStepTicks < 1 || directionSetupTicks < 0 ||
        directionSetupTicks > 65535 || !Math.isFinite(skewBound) || skewBound < 0.0)
      throw "Invalid virtual actuator configuration";
    for (i in 0...id.length)
      if (id.charCodeAt(i) < 33 || id.charCodeAt(i) > 126)
        throw "Virtual actuator ID must be printable ASCII";
    this.id = id;
    this.jointIndex = jointIndex;
    this.ratio = ratio;
    this.offset = offset;
    this.stepsPerUnit = stepsPerUnit;
    this.minStepTicks = minStepTicks;
    this.directionSetupTicks = directionSetupTicks;
    this.skewBound = skewBound;
    if (feedbackJointIndex < -1 || feedbackJointIndex >= 64 || !Math.isFinite(feedbackRatio) ||
        feedbackRatio == 0.0 || !Math.isFinite(feedbackOffset)) throw "Invalid physical feedback mapping";
    this.feedbackJointIndex = feedbackJointIndex;
    this.feedbackRatio = feedbackRatio;
    this.feedbackOffset = feedbackOffset;
  }
}

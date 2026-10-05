package robotkit.runtime;

/** Physical brake inputs in SI units, consumed only by the simulation adapter. */
class RobotRuntimeBrakePhysicsBlueprint {
  public final joint:Int;
  public final holdingEffort:Float;
  public final releaseDelay:Float;
  public final applicationDelay:Float;

  public function new(joint:Int, holdingEffort:Float, releaseDelay:Float, applicationDelay:Float) {
    if (joint < 0 || !Math.isFinite(holdingEffort) || holdingEffort <= 0.0 ||
        !Math.isFinite(releaseDelay) || releaseDelay < 0.0 ||
        !Math.isFinite(applicationDelay) || applicationDelay < 0.0)
      throw "Invalid physical brake parameters";
    this.joint = joint;
    this.holdingEffort = holdingEffort;
    this.releaseDelay = releaseDelay;
    this.applicationDelay = applicationDelay;
  }
}

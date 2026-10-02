package processkit;

import motionkit.path.OrientationPolicy;

/** Process limits and the analog output rate required per path speed. */
class ProcessRecipe {
  public final minSpeed:Float;
  public final maxSpeed:Float;
  public final nominalSpeed:Float;
  public final standoff:Float;
  public final orientationPolicy:OrientationPolicy;
  public final passSpacing:Float;
  public final quantityPerDistance:Float;
  public final triggerLeadSeconds:Float;
  public final recoveryBackoff:Float;
  public final feedChangePolicy:FeedChangePolicy;
  /** What engages the process before its path and disengages it after, or null for none. */
  public final engagement:Null<ProcessEngagement>;
  /** Feed of the move that brings the tool to the start of the path, or null for the process feed. */
  public final approachSpeed:Null<Float>;

  public function new(minSpeed:Float, maxSpeed:Float, nominalSpeed:Float,
      standoff:Float, orientationPolicy:OrientationPolicy, passSpacing:Float,
      quantityPerDistance:Float, triggerLeadSeconds:Float, recoveryBackoff:Float,
      feedChangePolicy:FeedChangePolicy, ?engagement:ProcessEngagement, ?approachSpeed:Float) {
    if (!Math.isFinite(minSpeed) || minSpeed <= 0.0 ||
        !Math.isFinite(maxSpeed) || maxSpeed < minSpeed ||
        !Math.isFinite(nominalSpeed) || nominalSpeed < minSpeed ||
        nominalSpeed > maxSpeed || !Math.isFinite(standoff) || standoff < 0.0 ||
        orientationPolicy == null || !Math.isFinite(passSpacing) ||
        passSpacing <= 0.0 || !Math.isFinite(quantityPerDistance) ||
        quantityPerDistance <= 0.0 || !Math.isFinite(triggerLeadSeconds) ||
        triggerLeadSeconds < 0.0 || !Math.isFinite(recoveryBackoff) ||
        recoveryBackoff < 0.0 || feedChangePolicy == null)
      throw "Process recipe needs finite positive speed, spacing, quantity and valid policy";
    if (approachSpeed != null && !(approachSpeed > 0.0 && Math.isFinite(approachSpeed)))
      throw "Process recipe approach speed must be finite and positive";
    this.engagement = engagement;
    this.approachSpeed = approachSpeed;
    this.minSpeed = minSpeed;
    this.maxSpeed = maxSpeed;
    this.nominalSpeed = nominalSpeed;
    this.standoff = standoff;
    this.orientationPolicy = orientationPolicy;
    this.passSpacing = passSpacing;
    this.quantityPerDistance = quantityPerDistance;
    this.triggerLeadSeconds = triggerLeadSeconds;
    this.recoveryBackoff = recoveryBackoff;
    this.feedChangePolicy = feedChangePolicy;
  }

  public function rateForSpeed(speed:Float):Float {
    if (!Math.isFinite(speed) || speed < minSpeed || speed > maxSpeed)
      throw 'Process speed $speed lies outside [$minSpeed, $maxSpeed]';
    return quantityPerDistance * speed;
  }
}

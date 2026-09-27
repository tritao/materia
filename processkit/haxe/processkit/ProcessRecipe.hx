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

  public function new(minSpeed:Float, maxSpeed:Float, nominalSpeed:Float,
      standoff:Float, orientationPolicy:OrientationPolicy, passSpacing:Float,
      quantityPerDistance:Float, triggerLeadSeconds:Float, recoveryBackoff:Float,
      feedChangePolicy:FeedChangePolicy) {
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

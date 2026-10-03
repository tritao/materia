package machinekit.motion;

/**
 * A gearbox between a motor and the joint it drives: the motor turns `ratio` times for one turn of the
 * joint, and the joint gets `efficiency` of the motor's power. Through it a joint's torque is the
 * motor's times the ratio times the efficiency, and its speed the motor's over the ratio, which is
 * how a joint's limits come from its drive rather than from numbers typed in with the joint. Drive-level
 * only: backlash and compliance are left out.
 */
class Gearbox {
  public final ratio:Float;
  public final efficiency:Float;

  public function new(ratio:Float, efficiency:Float) {
    if (!(ratio > 0.0) || !Math.isFinite(ratio)) throw "A gearbox needs a positive ratio";
    if (!(efficiency > 0.0 && efficiency <= 1.0)) throw "A gearbox's efficiency is above 0 and at most 1";
    this.ratio = ratio;
    this.efficiency = efficiency;
  }

  /** The joint's peak torque (N m) with a motor of `motorTorque`. */
  public function jointTorque(motorTorque:Float):Float return motorTorque * ratio * efficiency;

  /** The joint's top speed (rad/s) with a motor of `motorSpeed`. */
  public function jointSpeed(motorSpeed:Float):Float return motorSpeed / ratio;
}

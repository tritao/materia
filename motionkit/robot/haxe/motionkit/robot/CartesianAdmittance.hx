package motionkit.robot;

import motionkit.kinematics.Twist6;
import robotkit.spatial.Wrench3;

/**
 * Explicit six-axis virtual mass/damper admittance. `target` and the wrench
 * passed to `step` are the environment-on-tool wrench at the TCP, expressed in
 * the same caller-selected control axes. `step` returns a TCP velocity
 * correction in those axes; callers own sensor transforms, freshness, travel
 * limits and ServoSession deadlines.
 */
class CartesianAdmittance {
  public final target:Wrench3;
  final virtualMass:Array<Float>;
  final damping:Array<Float>;
  public final maxLinearSpeed:Float;
  public final maxAngularSpeed:Float;
  public final maxStepSeconds:Float;
  var velocity:Array<Float> = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

  /**
   * `virtualMass` is kg on translation axes and kg*m^2 on rotation axes;
   * `damping` is N*s/m and N*m*s/rad respectively. Every value is
   * commissioned input; no force-control gains or motion limits are guessed.
   */
  public function new(target:Wrench3, virtualMass:Array<Float>, damping:Array<Float>,
    maxLinearSpeed:Float, maxAngularSpeed:Float, maxStepSeconds:Float) {
    if (target == null || virtualMass == null || virtualMass.length != 6 || damping == null || damping.length != 6 ||
      !Math.isFinite(maxLinearSpeed) || !(maxLinearSpeed > 0) || !Math.isFinite(maxAngularSpeed) ||
      !(maxAngularSpeed > 0) || !Math.isFinite(maxStepSeconds) || !(maxStepSeconds > 0))
      throw "Cartesian admittance needs a target wrench, six explicit gains and positive motion bounds";
    for (index in 0...6) if (!Math.isFinite(virtualMass[index]) || !(virtualMass[index] > 0) ||
      !Math.isFinite(damping[index]) || !(damping[index] > 0))
      throw 'Cartesian admittance axis $index needs positive finite virtual mass and damping';
    this.target = target;
    this.virtualMass = virtualMass.copy();
    this.damping = damping.copy();
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
    this.maxStepSeconds = maxStepSeconds;
  }

  /** Integrate M*a + D*v = measured - target, with a commissioned stable sample bound. */
  public function step(measured:Wrench3, dt:Float):Twist6 {
    if (measured == null)
      throw "Cartesian admittance needs a measured wrench";
    // Robot clocks are nanosecond-valued; converting their delta to seconds can
    // put a commissioned decimal period one nanosecond over its Float value.
    if (!Math.isFinite(dt) || !(dt > 0) || dt - maxStepSeconds > 2e-9)
      throw 'Cartesian admittance timestep $dt s is outside its commissioned bound of $maxStepSeconds s';
    var stepSeconds = Math.min(dt, maxStepSeconds);
    var desired = components(target), observed = components(measured), next = velocity.copy();
    for (index in 0...6) {
      var decay = stepSeconds * damping[index] / virtualMass[index];
      if (!Math.isFinite(decay) || decay > 1.0)
        throw 'Cartesian admittance timestep is unstable on axis $index';
      var acceleration = (observed[index] - desired[index] - damping[index] * velocity[index]) / virtualMass[index];
      next[index] += acceleration * stepSeconds;
    }
    clampNorm(next, 0, maxLinearSpeed);
    clampNorm(next, 3, maxAngularSpeed);
    for (value in next) if (!Math.isFinite(value)) throw "Cartesian admittance produced a non-finite correction";
    velocity = next;
    return new Twist6(next[0], next[1], next[2], next[3], next[4], next[5]);
  }

  public function stop():Void velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

  function clampNorm(values:Array<Float>, start:Int, maximum:Float):Void {
    var norm = Math.sqrt(values[start] * values[start] + values[start + 1] * values[start + 1] + values[start + 2] * values[start + 2]);
    if (norm > maximum) {
      var scale = maximum / norm;
      for (index in start...(start + 3)) values[index] *= scale;
    }
  }

  static function components(wrench:Wrench3):Array<Float> return [wrench.force.x, wrench.force.y,
    wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z];
}

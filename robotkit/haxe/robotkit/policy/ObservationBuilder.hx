package robotkit.policy;

import robotkit.policy.PolicySpec.ObservationTerm;
import robotkit.policy.VelocityReference.VelocityCommand;

/** What a controller knows at one control tick: encoders, IMU and its own state. Never simulator truth. */
typedef Observed = {
  q:Array<Float>,
  dq:Array<Float>,
  /** Base angular velocity, rad/s, base frame (gyroscope). */
  angularVelocity:Array<Float>,
  /** Unit down vector, base frame (estimated from the IMU). */
  down:Array<Float>,
  command:VelocityCommand,
  lastAction:Array<Float>,
  /** Seconds on the gait clock. */
  gaitTime:Float
};

/**
 * Turns what the controller observes into the policy's input vector: the spec's
 * terms in order, each scaled, with the last `historyLength` observations
 * stacked oldest first (the first observation repeats until the history fills).
 * `q` and `dq` are the policy joints' values in policy order.
 */
class ObservationBuilder {
  final spec:PolicySpec;
  final history:Array<Array<Float>> = [];

  public function new(spec:PolicySpec) {
    this.spec = spec;
  }

  public function reset():Void history.resize(0);

  /** Size of the stacked input vector. */
  public function inputSize():Int return spec.observationSize() * spec.historyLength;

  /** Builds one observation, pushes it into the history and returns the stacked input. */
  public function build(observed:Observed):Array<Float> {
    var current:Array<Float> = [];
    for (term in spec.observation) appendTerm(current, term, observed);
    if (history.length == 0) for (_ in 0...spec.historyLength) history.push(current.copy());
    else {
      history.shift();
      history.push(current.copy());
    }
    var stacked:Array<Float> = [];
    for (frame in history) for (value in frame) stacked.push(value);
    return stacked;
  }

  function appendTerm(out:Array<Float>, term:ObservationTerm, observed:Observed):Void {
    var n = spec.joints.length;
    switch term {
      case AngularVelocity(scale): scaled(out, observed.angularVelocity, scale);
      case ProjectedGravity(scale): scaled(out, observed.down, scale);
      case VelocityCommand(scale): scaled(out, [observed.command.vx, observed.command.vy, observed.command.wz], scale);
      case JointPosition(scale): scaled(out, [for (i in 0...n) observed.q[i] - spec.defaultPose[i]], scale);
      case JointVelocity(scale): scaled(out, observed.dq, scale);
      case LastAction(scale): scaled(out, observed.lastAction, scale);
      case GaitPhase(period):
        var phase = (observed.gaitTime % period) / period;
        out.push(Math.sin(2.0 * Math.PI * phase));
        out.push(Math.cos(2.0 * Math.PI * phase));
    }
  }

  static function scaled(out:Array<Float>, values:Array<Float>, scale:Array<Float>):Void
    for (i in 0...values.length) out.push(values[i] * (scale.length == 1 ? scale[0] : scale[i]));
}

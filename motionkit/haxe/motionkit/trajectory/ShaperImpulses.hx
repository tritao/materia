package motionkit.trajectory;

/**
 * The impulses a joint's motion is convolved with: the shaped motion is the sum of the original motion delayed by each
 * delay and weighted by its amplitude. The amplitudes are positive and add up to 1, so the shaped motion stays within the
 * original's velocity and acceleration and only its timing changes: it ends `tail()` later and lags `meanDelay()` on
 * average.
 */
class ShaperImpulses {
  public final delays:Array<Float>;
  public final amplitudes:Array<Float>;

  public function new(delays:Array<Float>, amplitudes:Array<Float>) {
    if (delays.length < 1 || delays.length != amplitudes.length || delays.length > Trajectory.MAX_SHAPER_IMPULSES)
      throw "A shaper has between one and " + Trajectory.MAX_SHAPER_IMPULSES + " impulses, each with a delay and an amplitude";
    var total = 0.0;
    for (index in 0...delays.length) {
      if (!(delays[index] >= 0.0) || !Math.isFinite(delays[index]) || (index>0 && delays[index]<delays[index-1]) ||
          !(amplitudes[index] > 0.0) || !Math.isFinite(amplitudes[index]))
        throw "Shaper impulses need ordered nonnegative delays and finite positive amplitudes";
      total += amplitudes[index];
    }
    if (!Math.isFinite(total)) throw "Shaper amplitudes must have a finite sum";
    this.delays = delays.copy();
    this.amplitudes = [for (amplitude in amplitudes) amplitude / total];
  }

  /** Impulses that only delay by `seconds`: what a joint that is not shaped runs beside shaped ones, to stay in step. */
  public static function delay(seconds:Float):ShaperImpulses return new ShaperImpulses([seconds], [1.0]);

  /** Delay of the last impulse: the shaped motion ends this long after the original. */
  public function tail():Float return delays[delays.length - 1];

  /** The amplitude-weighted mean of the delays: how far the shaped motion lags the original on average. */
  public function meanDelay():Float {
    var sum = 0.0;
    for (index in 0...delays.length) sum += amplitudes[index] * delays[index];
    return sum;
  }

  /** The same impulses `seconds` later: the shaping lined up with another joint's. */
  public function later(seconds:Float):ShaperImpulses
    return new ShaperImpulses([for (value in delays) value + seconds], amplitudes);

  /**
    What is left of a sinusoid of `frequencyHz` and damping ratio `systemDamping` that the motion would excite, as a
    fraction of what the unshaped motion excites (Singhose's residual vibration: the magnitude of the impulses' phasors
    weighted by their amplitudes and the decay until the last impulse).
  **/
  public function residual(frequencyHz:Float, systemDamping:Float):Float {
    var omega = 2.0 * Math.PI * frequencyHz;
    var damped = omega * Math.sqrt(1.0 - systemDamping * systemDamping);
    var re = 0.0, im = 0.0;
    var last = tail();
    for (index in 0...delays.length) {
      var weight = amplitudes[index] * Math.exp(-systemDamping * omega * (last - delays[index]));
      re += weight * Math.cos(damped * delays[index]);
      im += weight * Math.sin(damped * delays[index]);
    }
    return Math.sqrt(re * re + im * im);
  }
  /** Upper bound over a continuous frequency band, not just sampled resonances.
   * Each interval's nearest endpoint is at most half its width away. The sum of the absolute phasor derivatives is a
   * Lipschitz bound, including decay; refine until its contribution is below accuracy. */
  public function residualBound(lower:Float,upper:Float,systemDamping:Float,accuracy:Float = 0.001):Float {
    if (!(lower>0.0) || upper<lower || !Math.isFinite(upper) || !Math.isFinite(systemDamping) || systemDamping<0.0 || systemDamping>=1.0 || !(accuracy>0.0) || !Math.isFinite(accuracy))
      throw "Residual band needs positive ordered frequencies, damping below one and positive accuracy";
    var root = Math.sqrt(1.0-systemDamping*systemDamping), derivative = 0.0, last = tail();
    for (index in 0...delays.length) {
      var time = delays[index], decayTime = systemDamping*(last-time);
      derivative += amplitudes[index]*2*Math.PI*Math.exp(-2*Math.PI*lower*decayTime)*
        Math.sqrt(decayTime*decayTime+root*root*time*time);
    }
    var refinement = Math.ceil((upper-lower)*derivative/(2*accuracy));
    if (!Math.isFinite(refinement) || refinement>65536) throw "Residual band requires excessive refinement";
    var steps = Std.int(Math.max(1,refinement));
    var measured = 0.0;
    for (index in 0...(steps+1)) measured = Math.max(measured,residual(lower+(upper-lower)*index/steps,systemDamping));
    return Math.min(1.0,measured+derivative*(upper-lower)/(2*steps)+1e-14);
  }

}

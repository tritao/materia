package motionkit.path;

/** A spatial weave; edge holds consume forward progress, leaving frequency unchanged. */
class WeaveProfile {
  public final pattern:WeavePattern;
  public final amplitude:Float;
  public final period:Float;
  public final hold:Float;
  final rise:Float;
  final fall:Float;

  /** Amplitude in metres, frequency in cycles/metre, edge dwell in seconds at nominal seam speed. */
  public function new(pattern:WeavePattern, amplitude:Float, cyclesPerMetre:Float,
      nominalSpeed:Float, ?edgeDwellSeconds:Float = 0.0) {
    if (pattern == null || !Math.isFinite(amplitude) || amplitude < 0.0 ||
        !Math.isFinite(cyclesPerMetre) || cyclesPerMetre <= 0.0 ||
        !Math.isFinite(nominalSpeed) || nominalSpeed <= 0.0 ||
        !Math.isFinite(edgeDwellSeconds) || edgeDwellSeconds < 0.0)
      throw "Weave requires finite amplitude, positive frequency/feed and nonnegative edge dwell";
    this.pattern = pattern; this.amplitude = amplitude;
    period = 1.0 / cyclesPerMetre;
    hold = nominalSpeed * edgeDwellSeconds;
    if (2.0 * hold >= period) throw "Weave edge holds consume the whole cycle";
    var moving = period - 2.0 * hold;
    rise = moving * (pattern == WeavePattern.Zigzag ? 0.75 : 0.5);
    fall = moving - rise;
  }

  public static function perSecond(pattern:WeavePattern, amplitude:Float, cyclesPerSecond:Float,
      nominalSpeed:Float, ?edgeDwellSeconds:Float = 0.0):WeaveProfile
    return new WeaveProfile(pattern, amplitude, cyclesPerSecond / nominalSpeed, nominalSpeed, edgeDwellSeconds);

  public static function perMillimetre(pattern:WeavePattern, amplitude:Float, cyclesPerMillimetre:Float,
      nominalSpeed:Float, ?edgeDwellSeconds:Float = 0.0):WeaveProfile
    return new WeaveProfile(pattern, amplitude, cyclesPerMillimetre * 1000.0, nominalSpeed, edgeDwellSeconds);

  /** Interior cycle joins, in global seam coordinates. */
  public function breaks(from:Float, to:Float):Array<Float> {
    var result:Array<Float> = [];
    var cycle = Std.int(Math.floor(from / period));
    while (cycle * period < to) {
      for (local in [0.0, rise, rise + hold, rise + hold + fall]) {
        var at = cycle * period + local;
        if (at > from + 1e-12 && at < to - 1e-12 &&
            (result.length == 0 || at > result[result.length - 1] + 1e-12)) result.push(at);
      }
      cycle++;
    }
    return result;
  }

  /** At a join choose the derivative arriving from the left or leaving to the right. */
  public function at(distance:Float, ?arriving:Bool = false):WeaveSample {
    if (!Math.isFinite(distance) || distance < 0.0) throw "Weave progress must be finite and nonnegative";
    var local = distance - Math.floor(distance / period) * period;
    for (join in [0.0, rise, rise + hold, rise + hold + fall, period])
      if (Math.abs(local - join) < 1e-12) local = join;
    if (arriving && local < 1e-12 && distance > 0.0) local = period;
    if (local < rise || (arriving && local <= rise)) return moving(local / rise, rise, true);
    if (local < rise + hold || (arriving && local <= rise + hold && hold > 0.0))
      return new WeaveSample(amplitude, 0.0, 0.0);
    if (local < rise + hold + fall || (arriving && local <= rise + hold + fall))
      return moving((local - rise - hold) / fall, fall, false);
    return new WeaveSample(-amplitude, 0.0, 0.0);
  }

  function moving(u:Float, length:Float, up:Bool):WeaveSample {
    var sign = up ? 1.0 : -1.0;
    if (pattern == WeavePattern.Sine) {
      var angle = Math.PI * u, rate = Math.PI / length;
      return new WeaveSample(-sign * amplitude * Math.cos(angle),
        sign * amplitude * Math.sin(angle) * rate,
        sign * amplitude * Math.cos(angle) * rate * rate);
    }
    return new WeaveSample(sign * amplitude * (2.0 * u - 1.0), sign * 2.0 * amplitude / length, 0.0);
  }
}

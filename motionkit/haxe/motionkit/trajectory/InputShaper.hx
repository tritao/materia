package motionkit.trajectory;

/** The impulse sequences input shaping offers (Singhose's names, with the formulas Klipper's shapers use). */
enum InputShaperKind {
  /** Zero vibration: two impulses, half a damped period apart. The shortest, the least robust to a wrong frequency. */
  ZV;
  /** Modified zero vibration: three impulses over three quarters of a period; leaves under 10 % of the vibration for a frequency 10 % off (ZV about twice that). */
  MZV;
  /** Extra insensitive: three impulses over a whole period, the longest tail; designed to keep vibration under 5 % across a band around the frequency. */
  EI;
}

/**
 * An input shaper: a short sequence of impulses that a motion is convolved with so that it excites little vibration in
 * a mode of the given frequency and damping. The frequency and damping ratio are the machine's data: the resonance of
 * the belt springs and the mass they carry.
 */
class InputShaper {
  public final kind:InputShaperKind;
  /** The mode to cancel, hertz. */
  public final frequency:Float;
  /** Its damping ratio (0 for an undamped mode, 0.1 is typical of a belt-driven carriage). */
  public final damping:Float;
  public final impulses:ShaperImpulses;

  /** Vibration the extra-insensitive shaper tolerates at its design frequency, as a fraction of the unshaped one. */
  static inline var EI_VIBRATION:Float = 0.05;

  /** A shaper by the name its kind goes by in a project (`zv`, `mzv`, `ei`). */
  public static function named(kind:String, frequency:Float, damping:Float):InputShaper {
    var found = switch kind {
      case "zv": InputShaperKind.ZV;
      case "mzv": InputShaperKind.MZV;
      case "ei": InputShaperKind.EI;
      default: throw 'Unknown input shaper "$kind"';
    };
    return new InputShaper(found, frequency, damping);
  }

  public function new(kind:InputShaperKind, frequency:Float, damping:Float = 0.1) {
    if (!(frequency > 0.0) || !Math.isFinite(frequency)) throw "An input shaper needs a positive frequency";
    if (!(damping >= 0.0) || !(damping < 1.0)) throw "An input shaper needs a damping ratio from 0 up to 1";
    this.kind = kind;
    this.frequency = frequency;
    this.damping = damping;
    var root = Math.sqrt(1.0 - damping * damping);
    var halfPeriod = 0.5 / (frequency * root);
    var decay = Math.exp(-damping * Math.PI / root);
    var times:Array<Float>, weights:Array<Float>;
    switch kind {
      case ZV:
        times = [0.0, halfPeriod];
        weights = [1.0, decay];
      case MZV:
        var first = 1.0 - 1.0 / Math.sqrt(2.0);
        times = [0.0, 0.75 * halfPeriod, 1.5 * halfPeriod];
        weights = [first, (Math.sqrt(2.0) - 1.0) * decay, first * decay * decay];
      case EI:
        var first = 0.25 * (1.0 + EI_VIBRATION);
        times = [0.0, halfPeriod, 2.0 * halfPeriod];
        weights = [first, 0.5 * (1.0 - EI_VIBRATION) * decay, first * decay * decay];
    }
    this.impulses = new ShaperImpulses(times, weights);
  }
}

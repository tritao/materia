package robotkit.model;

/**
 * What an encoder reports at one moment: its count, and how many index pulses it has passed since it
 * was powered up. `power` is the joint position it was powered up at, which an incremental encoder
 * counts from; an absolute one counts from the joint's zero.
 */
class EncoderReading {
  public final encoder:Encoder;
  /** Joint position the encoder was powered up at, in joint units. */
  public final power:Float;
  public var count(default, null):Float = 0.0;
  public var indexPulses(default, null):Int = 0;

  public function new(encoder:Encoder, power:Float) {
    this.encoder = encoder;
    this.power = power;
    sample(power);
  }

  /** Reads the encoder with its joint at `position`: the count is quantised to whole counts. */
  public function sample(position:Float):Void {
    var origin = encoder.kind == EncoderKind.Incremental ? power : 0.0;
    count = Math.fround((position - origin) * encoder.countsPerUnit);
    if (encoder.index) indexPulses = pulses(position);
  }

  /** The joint position this count stands for, the reading as the machine would use it. */
  public function position():Float {
    var origin = encoder.kind == EncoderKind.Incremental ? power : 0.0;
    return origin + count / encoder.countsPerUnit;
  }

  function pulses(position:Float):Int {
    var period = encoder.indexPeriod();
    return Std.int(Math.abs(Math.floor(position / period) - Math.floor(power / period)));
  }
}

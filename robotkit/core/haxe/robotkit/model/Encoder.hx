package robotkit.model;

/**
 * An encoder on a joint: a sensor that reads the joint's position in counts. Which joint it is on
 * says what it sees. On a motor's own joint it is motor-side: it sees the rotor, so lost steps and a
 * servo's following error, but not backlash or belt stretch. On a joint the load moves through a
 * drive it is load-side (a linear scale, or an encoder on the driven pulley): it sees where the load
 * is, stretch and backlash included. Drive-level only: counts are quantised, with no noise or latency.
 *
 * The runtime does not compile encoders; a simulation reads them from the joint positions it
 * reports (see `EncoderReading`), and a device reports its own counts.
 */
class Encoder {
  public final id:String;
  public final joint:JointId;
  public final kind:EncoderKind;
  /** Counts per unit of joint position: per radian on a turning joint, per metre on a sliding one. */
  public final countsPerUnit:Float;
  /**
   * Whether it has an index pulse: once a revolution on a turning joint, and at the reference mark
   * (the joint's zero) on a sliding one.
   */
  public final index:Bool;

  public function new(id:String, joint:JointId, kind:EncoderKind, countsPerUnit:Float, index:Bool = false) {
    if (id == null || StringTools.trim(id).length == 0 || joint == null || StringTools.trim(joint).length == 0)
      throw "An encoder needs an id and a joint";
    if (kind != EncoderKind.Incremental && kind != EncoderKind.Absolute) throw "An encoder is incremental or absolute";
    if (!(countsPerUnit > 0.0) || !Math.isFinite(countsPerUnit)) throw "An encoder needs a positive finite resolution";
    this.id = id;
    this.joint = joint;
    this.kind = kind;
    this.countsPerUnit = countsPerUnit;
    this.index = index;
  }

  /** An encoder of `counts` per revolution (quadrature counts, so four times the line count) on a turning joint. */
  public static function perRevolution(id:String, joint:JointId, kind:EncoderKind, counts:Float, index:Bool = false):Encoder
    return new Encoder(id, joint, kind, counts / (2.0 * Math.PI), index);

  /** A linear scale of `counts` per millimetre on a sliding joint. */
  public static function perMillimetre(id:String, joint:JointId, kind:EncoderKind, counts:Float, index:Bool = false):Encoder
    return new Encoder(id, joint, kind, counts * 1000.0, index);

  /** The size of one count, in joint units. */
  public function resolution():Float return 1.0 / countsPerUnit;

  /** Position in joint units between index pulses: a turn, or none for a single reference mark. */
  public function indexPeriod():Float return 2.0 * Math.PI;
}

package robotkit.material;

/** Application-level payload and lift envelope. Native runtime limits remain authoritative. */
class LoadLimits {
  public final maxMassKg:Float;
  public final maxLoadMomentKgMeters:Float;
  public final maxLiftHeightMeters:Float;

  public function new(maxMassKg:Float, maxLoadMomentKgMeters:Float,
      maxLiftHeightMeters:Float) {
    for (value in [maxMassKg, maxLoadMomentKgMeters, maxLiftHeightMeters])
      if (!Math.isFinite(value) || value <= 0.0)
        throw "Load limits must be finite and positive";
    this.maxMassKg = maxMassKg;
    this.maxLoadMomentKgMeters = maxLoadMomentKgMeters;
    this.maxLiftHeightMeters = maxLiftHeightMeters;
  }

  /** Returns a description when a requested lift state exceeds the configured envelope. */
  public function violation(payload:Null<Payload>, liftHeightMeters:Float):Null<String> {
    if (!Math.isFinite(liftHeightMeters) || liftHeightMeters < 0.0)
      return "lift height must be finite and non-negative";
    if (liftHeightMeters > maxLiftHeightMeters)
      return "requested lift height exceeds configured load limits";
    if (payload == null) return null;
    if (payload.massKg > maxMassKg)
      return "payload mass exceeds configured load limits";
    if (payload.massKg * payload.centerOfMassForwardMeters > maxLoadMomentKgMeters)
      return "payload load moment exceeds configured load limits";
    return null;
  }
}

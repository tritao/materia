package robotkit.material;

/** Named fork mechanism joint map and load envelope. */
class ForkConfig {
  public final lift:ForkAxisConfig;
  public final tilt:Null<ForkAxisConfig>;
  public final spread:Null<ForkAxisConfig>;
  public final loadLimits:LoadLimits;

  public function new(lift:ForkAxisConfig, loadLimits:LoadLimits,
      ?tilt:ForkAxisConfig, ?spread:ForkAxisConfig) {
    if (lift == null || loadLimits == null)
      throw "Fork configuration requires lift axis and load limits";
    var names = [lift.jointName];
    for (axis in [tilt, spread]) {
      if (axis != null) {
        if (names.indexOf(axis.jointName) >= 0)
          throw "Fork axes must map to distinct robot joints";
        names.push(axis.jointName);
      }
    }
    this.lift = lift;
    this.tilt = tilt;
    this.spread = spread;
    this.loadLimits = loadLimits;
  }
}

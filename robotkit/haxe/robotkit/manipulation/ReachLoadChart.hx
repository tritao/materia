package robotkit.manipulation;

import robotkit.spatial.Transform3;

/** Piecewise robot load chart keyed by 3D flange distance from the base.
 * More detailed vendor charts can implement RobotLoadChart directly. */
class ReachLoadChart implements RobotLoadChart {
  final bands:Array<ReachLoadBand>;

  public function new(bands:Array<ReachLoadBand>) {
    if (bands == null || bands.length == 0) throw "Reach load chart requires bands";
    var previous = 0.0;
    for (band in bands) {
      if (band == null || band.maxReachMeters <= previous)
        throw "Reach load chart bands must have strictly increasing reaches";
      previous = band.maxReachMeters;
    }
    this.bands = bands.copy();
  }

  public function limitsAt(q:Array<Float>, base_T_flange:Transform3):Null<RobotPayloadLimit> {
    if (base_T_flange == null) throw "Load chart requires a flange pose";
    var reach = base_T_flange.translation.norm();
    for (band in bands) if (reach <= band.maxReachMeters + 1e-12) return band.limits;
    return null;
  }
}

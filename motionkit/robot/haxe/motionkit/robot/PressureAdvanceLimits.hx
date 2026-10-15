package motionkit.robot;

/** Conservative planning bounds for the box average and endpoint taper used by `Trajectory.advanceJoint`.
 * The executed quintic approximation must still pass continuous trajectory validation.
 */
class PressureAdvanceLimits {
  public static function planning(velocity:Float, acceleration:Float, lead:Float, window:Float):
      {velocity:Float, acceleration:Float, jerk:Float} {
    if (!Math.isFinite(velocity) || velocity <= 0 || !Math.isFinite(acceleration) || acceleration <= 0 ||
        !Math.isFinite(lead) || lead < 0 || !Math.isFinite(window) || window <= 0)
      throw "Invalid pressure-advance drive limits or timing";
    // Match the native transform's nanosecond-rounded half window, including its one-nanosecond minimum.
    var w = 2 * Math.max(1, Math.floor(window * 0.5e9 + 0.5)) * 1e-9;
    // For s(x)=10x^3-15x^4+6x^5: max |s'|=15/8 and max |s''|=10/sqrt(3).
    // The taper is a product of two such ramps, including when their windows overlap.
    var first = 15.0 / (4.0 * w);
    var second = (20.0 / Math.sqrt(3.0) + 225.0 / 32.0) / (w * w);
    // The box average m has |m|<=V, |m'|<=min(A,2V/w), |m''|<=2A/w.
    // Thus (q+K*m*taper)' and '' obey the following bounds even at acceleration jumps.
    var rate = Math.min(acceleration, 2 * velocity / w);
    var advancedVelocity = velocity + lead * (rate + velocity * first);
    var advancedAcceleration = acceleration + lead *
      (2 * acceleration / w + 2 * rate * first + velocity * second);
    var scale = Math.min(velocity / advancedVelocity, acceleration / advancedAcceleration);
    if (!Math.isFinite(scale) || scale <= 0) throw "Pressure-advance planning bounds overflow";
    // Scale both bounds together: every term above is positively homogeneous in V and A.
    // A/W sets the nominal acceleration transition time to the smoothing window; it is not a motor jerk claim.
    return {velocity:velocity * scale, acceleration:acceleration * scale, jerk:acceleration * scale / w};
  }
}

package machinekit.sizing;

import machinekit.motion.ServoMotor.ServoRating;

/**
 * The crash protection one linear axis's servo drive is set to: what the drive's torque limit and following-error trip must be so that a jam
 * (a stop hit, a collision, a stuck axis) loads the structure with a bounded thrust and not the motor's peak.
 *
 * - `torqueLimit` is the motor torque the drive will not exceed, N m (a drive parameter, set as a share of the motor's rated torque in steps of `STEP`).
 * - `thrust` is the axis thrust that torque makes at the nut (`thrustPerNm`, the screw's lead and efficiency), the load the joints are checked for in a crash.
 * - `followingErrorTrip` is the position error, m, at which the drive faults and cuts the torque: a jam shows as a growing error, so the trip bounds the
 *   energy a limited thrust can put into the stop (`thrust` x `followingErrorTrip`, reported as `jamEnergy`, J).
 *
 * The rule: the limit is the largest multiple of `STEP` of the rated torque that keeps the thrust at or under `DESIGN_FACTOR` times the sizing thrust
 * (the duty of `LinearDriveSizing`: acceleration, process force, friction and gravity, with its reserve), and not under `MIN_HEADROOM` times it, so the
 * limit never clips the axis's own motion. Both factors are choices of the machine's designer, here as constants; the trip is ASSUMED
 * (`FOLLOWING_ERROR_TRIP`): it must be set above the largest following error the tuned axis shows at its rapid speed, and the drive's own parameter
 * names and ranges are the drive maker's manual's, not the model's.
 */
typedef CrashLimit = {
  var designTorque:Float;
  var designThrust:Float;
  var torqueLimit:Float;
  /** The torque limit as a share of the motor's rated torque (1.0 is the rated torque). */
  var ratedShare:Float;
  var thrust:Float;
  /** Limited thrust over the sizing thrust. */
  var factorOnDesign:Float;
  /** Limited thrust over the motor's peak thrust. */
  var shareOfPeak:Float;
  var followingErrorTrip:Float;
  /** Work done on the stop up to the trip, J. */
  var jamEnergy:Float;
}

class CrashProtection {
  /** The limited thrust is at most this many times the sizing thrust. */
  public static inline final DESIGN_FACTOR = 2.0;
  /** ... and at least this many times: below it the limit would clip the sizing duty's own accelerations. */
  public static inline final MIN_HEADROOM = 1.5;
  /** The drive parameter's resolution: a share of the motor's rated torque. */
  public static inline final STEP = 0.05;
  /** ASSUMED following-error trip, m (no tuned axis exists): 1 mm of axis travel, 0.2 screw turns on the 5 mm lead. */
  public static inline final FOLLOWING_ERROR_TRIP = 0.001;

  /** The crash protection for an axis whose sizing demands `designTorque` N m at the motor, `thrustPerNm` N of axis thrust per N m. */
  public static function limit(designTorque:Float, thrustPerNm:Float, motor:ServoRating, trip:Float = FOLLOWING_ERROR_TRIP):CrashLimit {
    if (!(designTorque > 0) || !(thrustPerNm > 0) || !(trip > 0) || !Math.isFinite(designTorque + thrustPerNm + trip)) throw "Crash protection needs a positive sizing torque, thrust per torque and trip";
    var step = STEP * motor.ratedTorque, steps = Math.floor(DESIGN_FACTOR * designTorque / step + 1e-9), torque = steps * step;
    if (torque < MIN_HEADROOM * designTorque - 1e-9) throw 'Crash protection: no torque limit in steps of ${STEP * 100} % of rated holds ${MIN_HEADROOM} x the sizing torque ${designTorque} N m'
      + ' and stays under ${DESIGN_FACTOR} x it';
    if (torque > motor.peakTorque + 1e-9) throw "Crash protection: the torque limit exceeds the motor's peak torque";
    var thrust = torque * thrustPerNm;
    return {
      designTorque: designTorque, designThrust: designTorque * thrustPerNm, torqueLimit: torque, ratedShare: torque / motor.ratedTorque, thrust: thrust,
      factorOnDesign: torque / designTorque, shareOfPeak: torque / motor.peakTorque, followingErrorTrip: trip, jamEnergy: thrust * trip
    };
  }

  /** One line for a report. */
  public static function describe(axis:String, c:CrashLimit):String {
    function r(v:Float, places:Float = 10):Float return Math.round(v * places) / places;
    return '$axis: torque limit ${r(c.torqueLimit, 1000)} N m (${r(c.ratedShare * 100, 1)} % of rated, ${r(c.shareOfPeak * 100, 1)} % of peak) = thrust ${Math.round(c.thrust)} N, '
      + '${r(c.factorOnDesign, 100)} x the sizing thrust ${Math.round(c.designThrust)} N; following-error trip ${r(c.followingErrorTrip * 1000, 100)} mm (jam energy ${r(c.jamEnergy, 100)} J)';
  }
}

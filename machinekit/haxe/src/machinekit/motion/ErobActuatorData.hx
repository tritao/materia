package machinekit.motion;

import machinekit.transmission.ValueBasis;

/** One harmonic-drive ratio of the actuator: torques in N m at the output flange, speed in output rpm. */
typedef ErobRatioRow = {
  var ratio:Int;
  /** Peak torque for repeated start-stop duty. */
  var peakTorque:Float;
  /** Highest permitted average load torque. */
  var averageTorque:Float;
  var ratedTorque:Float;
  /** Momentary (collision / emergency stop) torque limit. */
  var momentaryTorque:Float;
  /** Output speed at rated torque on a 48 V bus. */
  var speedRpm:Float;
}

/** One input the maker has not published, or published only as a grade, with where it stands. */
typedef ErobInput = {
  var name:String;
  var value:Null<Float>;
  var unit:String;
  var basis:ValueBasis;
  var reason:String;
}

/** One encoder grade of the output-side absolute encoder; accuracies in arc seconds. */
typedef ErobEncoderGrade = {
  var bits:Int;
  var repeatability:Float;
  var absolute:Float;
}

/**
 * ZeroErr eRob 110 "I" series integrated strain-wave actuator, V6 revision D, MC2E.DX (EtherCAT, 18 mm hollow
 * bore): every number is from the maker's drawing `eRob110H x x I-XX-18EX_V6_MC2E.DX.XX.pdf`, read on 2026-11-03.
 * Lengths mm, torques N m, motor values at the motor shaft.
 */
class ErobActuatorData {
  public static inline final SOURCE = "ZeroErr eRob110I-18EX V6 MC2E.DX drawing (eRob110H x x I-XX-18EX_V6_MC2E.DX.XX.pdf)";
  /** SHA-256 of the maker's STEP file this data was read beside (`eRob110H x x I-XX-18EX_V6_MC2E.DX.XX.step`). */
  public static inline final STEP_SHA256 = "81eaf3294a6d38ed70cc42d6a0bca8718f3eacf76f7c5adc0edd054ee0c166ad";

  public static inline final BODY_DIAMETER = 110.0;
  public static inline final BODY_DIAMETER_TOLERANCE = 0.035;
  /** Rear face to output mounting face. */
  public static inline final BODY_LENGTH = 80.2;
  /** Including the rear connectors. */
  public static inline final OVERALL_LENGTH = 83.6;
  public static inline final BORE_DIAMETER = 18.0;
  /** Fixed housing: M4 holes in the front face, on this pitch circle at this spacing. */
  public static inline final FIXED_PITCH_CIRCLE = 102.0;
  public static inline final FIXED_BOLT_COUNT = 16;
  /** The pocket that receives the housing's front end is at least this deep. */
  public static inline final POCKET_DEPTH_MIN = 8.0;
  /** Output flange: M4 holes on this pitch circle. */
  public static inline final OUTPUT_PITCH_CIRCLE = 77.0;
  public static inline final OUTPUT_BOLT_COUNT = 12;
  public static inline final OUTPUT_PILOT_DIAMETER = 85.0;
  public static inline final OUTPUT_PILOT_TOLERANCE = 0.022;
  /** Clearance hole the drawing gives the mating part for the output M4 screws. */
  public static inline final OUTPUT_CLEARANCE_HOLE = 4.6;
  /** M4 thread depth in the output flange. */
  public static inline final OUTPUT_THREAD_DEPTH = 6.0;
  /** Rear spigot. */
  public static inline final REAR_SPIGOT_DIAMETER = 85.0;
  /** Output pilot height and flange stand-off above the housing front face, from the section view; the drawing does not dimension either on its own. */
  public static inline final PILOT_HEIGHT = 6.0;
  public static inline final REAR_SPIGOT_HEIGHT = 2.0;
  public static inline final FIXED_THREAD_DEPTH = 7.0;
  /** Rev D output-flange locking screws sit on the output pitch circle; the drawing neither dimensions nor counts them. */
  public static inline final LOCKING_SCREW_NOTE = "Rev D output-flange locking screws: position and count not dimensioned on the drawing";

  public static inline final MASS = 2.57;
  public static inline final MASS_BRAKED = 2.68;
  /** Rotor inertia at the motor shaft, kg m2 (308632 g mm2, 316600 braked). */
  public static inline final ROTOR_INERTIA = 308632e-9;
  public static inline final ROTOR_INERTIA_BRAKED = 316600e-9;

  public static inline final BUS_VOLTS = 48.0;
  public static inline final BUS_TOLERANCE = 0.10;
  public static inline final POLES = 16;
  public static inline final RATED_POWER = 723.0;
  public static inline final MOTOR_RATED_TORQUE = 2.3;
  public static inline final MOTOR_RATED_RPM = 3000.0;
  public static inline final MOTOR_RATED_CURRENT = 18.9;
  public static inline final MOTOR_MAX_TORQUE = 6.9;
  public static inline final MOTOR_MAX_CURRENT = 56.7;
  public static inline final MOTOR_STALL_TORQUE = 2.53;
  public static inline final MOTOR_STALL_CURRENT = 20.79;
  public static inline final TORQUE_CONSTANT = 0.134;
  /** V per 1000 rpm. */
  public static inline final BACK_EMF = 8.1;
  public static inline final LINE_RESISTANCE = 0.07;
  public static inline final LINE_INDUCTANCE_MH = 0.21;

  public static function ratios():Array<ErobRatioRow> return [
    {ratio: 50, peakTorque: 127, averageTorque: 72, ratedTorque: 51, momentaryTorque: 242, speedRpm: 60},
    {ratio: 80, peakTorque: 178, averageTorque: 113, ratedTorque: 82, momentaryTorque: 332, speedRpm: 37.5},
    {ratio: 100, peakTorque: 204, averageTorque: 140, ratedTorque: 87, momentaryTorque: 369, speedRpm: 30},
    {ratio: 120, peakTorque: 217, averageTorque: 140, ratedTorque: 87, momentaryTorque: 395, speedRpm: 25},
    {ratio: 160, peakTorque: 229, averageTorque: 140, ratedTorque: 87, momentaryTorque: 408, speedRpm: 18.75}
  ];

  public static function row(ratio:Int):ErobRatioRow {
    for (r in ratios()) if (r.ratio == ratio) return r;
    throw 'eRob 110 has no $ratio:1 ratio; offered: ${[for (r in ratios()) r.ratio].join(", ")}';
  }

  public static function encoderGrades():Array<ErobEncoderGrade> return [
    {bits: 19, repeatability: 10, absolute: 25},
    {bits: 20, repeatability: 7, absolute: 15}
  ];

  public static function encoderGrade(bits:Int):ErobEncoderGrade {
    for (g in encoderGrades()) if (g.bits == bits) return g;
    throw 'eRob 110 has no $bits-bit output encoder; offered: 19, 20';
  }

  /**
   * What the maker has not published. The values are unset until an assumption is entered as a recipe input
   * (`momentRating`, `brakeHoldingTorque`, `bearingStiffness`); once entered the input is `Assumed` and carries
   * its stated reason. Asked from ZeroErr 2026-11-03.
   */
  public static function unpublished():Array<{name:String, unit:String, reason:String}> return [
    {name: "momentRating", unit: "N m",
      reason: "Output bearing permissible overturning moment is not on the drawing; asked from ZeroErr"},
    {name: "bearingStiffness", unit: "N m/rad",
      reason: "Output bearing tilt stiffness is not on the drawing; asked from ZeroErr"},
    {name: "brakeHoldingTorque", unit: "N m",
      reason: "Friction brake holding torque is not on the drawing; asked from ZeroErr"}
  ];
}

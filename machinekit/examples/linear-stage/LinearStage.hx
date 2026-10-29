import machinekit.assembly.LinearAxis;
import machinekit.operator.OperatorAxis;
import machinekit.operator.OperatorKey;
import machinekit.operator.OperatorProfile;

/**
 * A single-axis lead-screw stage: a NEMA 23 stepper turning a 10 mm trapezoidal screw (2 mm lead) through a coupling, between two
 * flange bearings, with a carriage on a profile rail (MGN12C) and 200 mm of travel. It is the library's `LinearAxis` as it is, so its
 * joint, limits, screw and bearings are the real ones; nothing here is drawn for the example.
 */
class LinearStage {
  public static inline var TRAVEL:Float = 200;
  /** The carriage's joint in the axis. */
  public static inline var JOINT:String = "carriage-slide";

  public static function axis():LinearAxis return LinearAxis.forRailProfile("MGN12C", 23, 10, TRAVEL);

  /** The operator profile of a one-axis stage: X, jogged on the pendant's first position. */
  public static function operator():OperatorProfile return new OperatorProfile(
    "linear-stage",
    [new OperatorAxis("x", "X")],
    ["x", null, null, null, null, null, null, null, null, null, null],
    [new OperatorKey(Unused, ""), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")],
    [Jog],
    false,
    false,
    []
  );

  /** Which joint a jog of the profile's axis moves. */
  public static function axisJoints():Map<String, String> return ["x" => JOINT];
}

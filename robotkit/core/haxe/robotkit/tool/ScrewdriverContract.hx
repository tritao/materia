package robotkit.tool;

/** Device-neutral sensor units: Nm, radians, rad/s, mm, phase, result and digital vacuum. */
class ScrewdriverContract {
  public static inline var SENSOR_KIND:String = "tool_screwdriver";
  public static inline var TORQUE:Int = 0;
  public static inline var ANGLE:Int = 1;
  public static inline var SPEED:Int = 2;
  public static inline var STROKE:Int = 3;
  public static inline var PHASE:Int = 4;
  public static inline var RESULT:Int = 5;
  public static inline var VACUUM:Int = 6;
  public static inline var SENSOR_COUNT:Int = 7;
  public static inline var IDLE:Int = 0;
  public static inline var ENGAGE:Int = 1;
  public static inline var RUN_DOWN:Int = 2;
  public static inline var SNUG:Int = 3;
  public static inline var FINAL:Int = 4;
  public static inline var COMPLETE:Int = 5;
  public static inline var NONE:Int = 0;
  public static inline var OK:Int = 1;
  public static inline var NO_SCREW:Int = 2;
  public static inline var NO_START:Int = 3;
  public static inline var CROSS_THREAD:Int = 4;
  public static inline var STRIPPED:Int = 5;
  public static inline var EARLY_HIGH_TORQUE:Int = 6;
  public static inline var ANGLE_OUT_OF_WINDOW:Int = 7;
  public static inline var RUN_DOWN_TOO_LONG:Int = 8;
  public static inline var SCREW_LOST:Int = 9;
  /** Finite physical values, integral known phase/result codes and a digital vacuum flag. */
  public static function valid(values:Array<Float>):Bool {
    if (values == null || values.length != SENSOR_COUNT) return false;
    for (value in values) if (!Math.isFinite(value)) return false;
    return values[TORQUE] >= 0 && values[ANGLE] >= 0 && values[STROKE] >= 0 &&
      values[PHASE] == Math.floor(values[PHASE]) && values[PHASE] >= IDLE && values[PHASE] <= COMPLETE &&
      values[RESULT] == Math.floor(values[RESULT]) && values[RESULT] >= NONE && values[RESULT] <= SCREW_LOST &&
      (values[VACUUM] == 0 || values[VACUUM] == 1);
  }
}

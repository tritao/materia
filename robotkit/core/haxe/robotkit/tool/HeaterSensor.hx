package robotkit.tool;

/** One reading of a heater: what a `tool_heater` sensor frame carries. */
typedef HeaterReading = {
  /** The thermistor's temperature in degrees Celsius; 0 when it cannot be read (a fault says why). */
  var celsius:Float;
  /** The target the controller is holding, 0 when the heater is off. */
  var targetCelsius:Float;
  /** The fraction of the supply applied, 0 to 1. */
  var duty:Float;
  /** The temperature is within the controller's band of a target above zero. */
  var atTarget:Bool;
  /** A `HeaterSensor.faultMessage` code; 0 is none. */
  var fault:Int;
};

/**
 * The `tool_heater` sensor: one frame of five values in a fixed order, so a reading is one consistent snapshot of
 * the controller. The controller owns the heater (PID, a runaway watchdog, a safe state): the host sets a target on
 * the heater's analog channel and reads this frame back.
 */
class HeaterSensor {
  public static inline var KIND:String = "tool_heater";
  public static inline var TEMPERATURE:Int = 0;
  public static inline var TARGET:Int = 1;
  public static inline var DUTY:Int = 2;
  public static inline var AT_TARGET:Int = 3;
  public static inline var FAULT:Int = 4;
  public static inline var COUNT:Int = 5;

  /** Fault codes, as the controller latches them. */
  public static inline var SENSOR_OPEN:Int = 1;
  public static inline var SENSOR_SHORT:Int = 2;
  public static inline var OVER_TEMPERATURE:Int = 3;
  public static inline var NOT_HEATING:Int = 4;
  public static inline var LEFT_BAND:Int = 5;
  public static inline var COLD_EXTRUSION:Int = 6;
  public static inline var INVALID:Int = 7;

  public static function values(reading:HeaterReading):Array<Float>
    return [reading.celsius, reading.targetCelsius, reading.duty, reading.atTarget ? 1.0 : 0.0, reading.fault];

  /** The fault code in words, or null for none. */
  public static function faultMessage(code:Int):Null<String> {
    return switch code {
      case 0: null;
      case 1: "heater: the thermistor reads open (unplugged or broken)";
      case 2: "heater: the thermistor reads shorted";
      case 3: "heater: over temperature";
      case 4: "heater: thermal runaway, it is not heating (the heater or thermistor is disconnected, or a draught takes the heat)";
      case 5: "heater: thermal runaway, it left its target and did not come back";
      case 6: "heater: cold extrusion, the extruder pushed filament below the minimum temperature";
      case 7: "heater: invalid configuration or command";
      default: 'heater: unknown fault $code';
    }
  }

  public static function reading(values:Array<Float>):HeaterReading {
    if (values == null || values.length != COUNT) throw 'A tool_heater frame has $COUNT values';
    return {celsius: values[TEMPERATURE], targetCelsius: values[TARGET], duty: values[DUTY],
      atTarget: values[AT_TARGET] != 0.0, fault: Std.int(values[FAULT])};
  }

  /** Whether `values` is a well-formed frame: five finite values, a duty from 0 to 1, a flag of 0 or 1, a known fault code. */
  public static function valid(values:Array<Float>):Bool {
    if (values == null || values.length != COUNT) return false;
    for (value in values) if (!Math.isFinite(value)) return false;
    return values[TARGET] >= 0.0 && values[DUTY] >= 0.0 && values[DUTY] <= 1.0 &&
      (values[AT_TARGET] == 0.0 || values[AT_TARGET] == 1.0) && values[FAULT] == Std.int(values[FAULT]) &&
      values[FAULT] >= 0.0 && values[FAULT] <= 7.0;
  }
}

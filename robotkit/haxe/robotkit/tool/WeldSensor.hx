package robotkit.tool;

/** One reading of a welder: what a `tool_weld` sensor frame carries. */
typedef WeldReading = {
  /** The arc is established. */
  var arc:Bool;
  /** Welding current, in amperes. */
  var currentA:Float;
  /** Voltage across the arc, in volts: the arc voltage while it burns, the sensing voltage while it is off (it collapses on touch). */
  var voltageV:Float;
  /** The wire touches the work with the arc off. */
  var touch:Bool;
  /** A `WeldFault` code; 0 is none. */
  var fault:Int;
  /** Power the supply draws from the mains, in watts. */
  var powerW:Float;
};

/**
 * The `tool_weld` sensor: one frame of six values in a fixed order, so a reading is a consistent snapshot of
 * the circuit rather than six sensors sampled at different moments. A real welder (a retrofit I/O board or
 * a Modbus supply) publishes the same frame.
 */
class WeldSensor {
  public static inline var KIND:String = "tool_weld";
  public static inline var ARC:Int = 0;
  public static inline var CURRENT:Int = 1;
  public static inline var VOLTAGE:Int = 2;
  public static inline var TOUCH:Int = 3;
  public static inline var FAULT:Int = 4;
  public static inline var POWER:Int = 5;
  public static inline var COUNT:Int = 6;

  public static function values(reading:WeldReading):Array<Float>
    return [reading.arc ? 1.0 : 0.0, reading.currentA, reading.voltageV, reading.touch ? 1.0 : 0.0, reading.fault, reading.powerW];

  /** The fault code in words, or null for none. */
  public static function faultMessage(code:Int):Null<String> {
    return switch code {
      case 0: null;
      case 1: "weld: the arc did not ignite";
      case 2: "weld: the arc was lost";
      case 3: "weld: the wire is stuck to the work";
      default: 'weld: unknown fault $code';
    }
  }

  public static function reading(values:Array<Float>):WeldReading {
    if (values == null || values.length != COUNT) throw 'A tool_weld frame has $COUNT values';
    return {arc: values[ARC] != 0.0, currentA: values[CURRENT], voltageV: values[VOLTAGE], touch: values[TOUCH] != 0.0,
      fault: Std.int(values[FAULT]), powerW: values[POWER]};
  }

  /** Whether `values` is a well-formed frame: six finite values, flags 0 or 1, a known fault code, no negative current or power. */
  public static function valid(values:Array<Float>):Bool {
    if (values == null || values.length != COUNT) return false;
    for (value in values) if (!Math.isFinite(value)) return false;
    return (values[ARC] == 0.0 || values[ARC] == 1.0) && (values[TOUCH] == 0.0 || values[TOUCH] == 1.0) &&
      values[CURRENT] >= 0.0 && values[POWER] >= 0.0 && values[FAULT] == Std.int(values[FAULT]) &&
      values[FAULT] >= 0.0 && values[FAULT] <= 3.0;
  }
}

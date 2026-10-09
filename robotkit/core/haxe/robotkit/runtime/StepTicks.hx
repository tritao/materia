package robotkit.runtime;

/**
 * The least whole number of step-tick periods between two steps: what an
 * RKD6 device enforces (`min_step_ticks`). The host chooses it once, from
 * a step frequency it actually has (an actuator's authored speed in steps,
 * a driver's step-rate ceiling), and derives every rate from it; nothing
 * turns a rate back into ticks, so float rounding can never cost a tick.
 */
class StepTicks {
  /** The ticks for at most `stepsPerSecond` (null: no limit but the tick itself). */
  public static function forLimit(stepsPerSecond:Null<Float>, tickHz:Int):Int {
    if (tickHz <= 0) throw "Step ticks need a positive tick rate";
    if (stepsPerSecond == null || stepsPerSecond >= tickHz) return 1;
    if (!(stepsPerSecond > 0) || !Math.isFinite(stepsPerSecond)) throw "A step limit must be finite and positive";
    return Std.int(Math.ceil(tickHz / stepsPerSecond));
  }

  /** The rate, in units per second, a device stepping every `ticks` reaches with its f32 `stepsPerUnit`. */
  public static function rate(ticks:Int, stepsPerUnit:Float, tickHz:Int):Float
    return tickHz / (ticks * deployed(stepsPerUnit));

  /** Steps per unit as the device holds them (f32 on the wire). */
  public static function deployed(stepsPerUnit:Float):Float {
    var wire = haxe.io.Bytes.alloc(4);
    wire.setFloat(0, stepsPerUnit);
    return wire.getFloat(0);
  }
}

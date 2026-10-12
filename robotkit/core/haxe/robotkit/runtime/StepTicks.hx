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

  /** The rate, in units per second, a device stepping every `ticks` reaches with `stepsPerUnit` (already `wire`). */
  public static function rate(ticks:Int, stepsPerUnit:Float, tickHz:Int):Float
    return tickHz / (ticks * stepsPerUnit);

  /**
   * Steps per unit as the wire carries them (f32). The binding quantises its steps per unit here once and
   * plans, deploys and checks with that value, so no later stage has to mirror the device's rounding.
   */
  public static function wire(stepsPerUnit:Float):Float {
    var wire = haxe.io.Bytes.alloc(4);
    wire.setFloat(0, stepsPerUnit);
    return wire.getFloat(0);
  }
}

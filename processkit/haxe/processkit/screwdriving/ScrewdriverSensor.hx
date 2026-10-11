package processkit.screwdriving;

import robotkit.tool.ScrewdriverContract;

/** Seven values in the canonical screwdriver contract. */
class ScrewdriverSensor {
  public static function idle():ScrewdriverReading return {torqueNm:0, angle:0, speed:0, strokeMm:0,
    phase:ScrewdriverContract.IDLE, result:ScrewdriverContract.NONE, vacuum:false};
  public static function values(reading:ScrewdriverReading):Array<Float> return [reading.torqueNm, reading.angle,
    reading.speed, reading.strokeMm, reading.phase, reading.result, reading.vacuum ? 1.0 : 0.0];
  public static function valid(values:Array<Float>):Bool return ScrewdriverContract.valid(values);
  public static function reading(values:Array<Float>):ScrewdriverReading {
    if (!valid(values)) throw "Screwdriver sensor needs a valid seven-value frame";
    return {torqueNm: values[ScrewdriverContract.TORQUE], angle:values[ScrewdriverContract.ANGLE], speed:values[ScrewdriverContract.SPEED], strokeMm:values[ScrewdriverContract.STROKE], phase:Std.int(values[ScrewdriverContract.PHASE]),
      result:Std.int(values[ScrewdriverContract.RESULT]), vacuum:values[ScrewdriverContract.VACUUM] != 0};
  }
  public static function faultMessage(code:Int):Null<String> return switch code {
    case ScrewdriverContract.NONE, ScrewdriverContract.OK: null;
    case ScrewdriverContract.NO_SCREW: "screwdriver: no screw";
    case ScrewdriverContract.NO_START: "screwdriver: no start";
    case ScrewdriverContract.CROSS_THREAD: "screwdriver: cross-thread";
    case ScrewdriverContract.STRIPPED: "screwdriver: stripped thread";
    case ScrewdriverContract.EARLY_HIGH_TORQUE: "screwdriver: early high torque";
    case ScrewdriverContract.ANGLE_OUT_OF_WINDOW: "screwdriver: angle out of window";
    case ScrewdriverContract.RUN_DOWN_TOO_LONG: "screwdriver: run-down too long";
    case ScrewdriverContract.SCREW_LOST: "screwdriver: screw lost";
    default: 'screwdriver: unknown result $code';
  };
}

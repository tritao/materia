package processkit.screwdriving;

import haxe.Int64;

/** One atomic I/O and measured-process acquisition in the owner's declared clock.
    Values use ScrewdriverSensor's seven-value SI contract, including a cumulative
    spindle angle with the commissioned engagement datum, not an angle-from-snug result. */
typedef ScrewdriverIoSample = {
  var sequence:Int64;
  var acquiredAt:Float;
  var clockId:String;
  /** Controller-acquired program echo, in the commissioned binary code space. */
  var selectedProgramCode:Int;
  var inputs:Array<Bool>;
  var values:Array<Float>;
}

/** Commissioned digital control plus measured feedback; implementations own their watchdog. */
interface ScrewdriverIo {
  function write(output:Int, value:Bool):Void;
  function sample():Null<ScrewdriverIoSample>;
}

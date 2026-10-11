package processkit.feeding;

import haxe.Int64;

typedef ScrewFeederIoSample = {
  var sequence:Int64;
  var acquiredAt:Float;
  var clockId:String;
  var inputs:Array<Bool>;
};

/** The host supplies atomic acquisitions and owns physical polarity/watchdogs. */
interface ScrewFeederIo {
  function write(pin:Int, on:Bool):Void;
  function sample():Null<ScrewFeederIoSample>;
}

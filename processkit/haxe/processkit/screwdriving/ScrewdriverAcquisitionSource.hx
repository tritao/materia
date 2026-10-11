package processkit.screwdriving;

import haxe.Int64;

/** One validated acquisition, retaining the provider's sequence and source clock. */
typedef ScrewdriverAcquisition = {
  var sequence:Int64;
  var acquiredAt:Float;
  var clockId:String;
  var values:Array<Float>;
}

/** Measured frames are copied, never refreshed by reading them again. */
interface ScrewdriverAcquisitionSource {
  function acquisition():Null<ScrewdriverAcquisition>;
}

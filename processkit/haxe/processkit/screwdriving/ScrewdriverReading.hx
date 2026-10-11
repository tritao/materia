package processkit.screwdriving;
/** A consistent sample of the screwdriver process. */
typedef ScrewdriverReading = {
  var torqueNm:Float;
  var angle:Float;
  var speed:Float;
  var strokeMm:Float;
  var phase:Int;
  var result:Int;
  var vacuum:Bool;
}

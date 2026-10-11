package processkit.feeding;

import haxe.Int64;

/** Atomic source observations; absent empty/jam sensors remain unknown. */
typedef ScrewFeederReading = {
  var sequence:Int64;
  var acquiredAt:Float;
  var clockId:String;
  var ready:Bool;
  var present:Bool;
  var empty:Null<Bool>;
  var jammed:Null<Bool>;
};

/** One separated screw per request, with an occupied-nest interlock. */
interface ScrewFeederDevice {
  function enable(on:Bool):Void;
  function request():Void;
  function poll(now:Float):Void;
  function reading():Null<ScrewFeederReading>;
  function safe():Void;
  function faultDetail():Null<String>;
}

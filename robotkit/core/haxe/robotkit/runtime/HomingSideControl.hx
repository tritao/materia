package robotkit.runtime;

/** Motor-side holds for a homing owner. Holds use physical shaft coordinates. */
interface HomingSideControl {
  function beginSquaring(switchIds:Array<String>):Void;
  function endSquaring():Void;
  function hold(switchId:String):Void;
  function leaderCapture(switchId:String, sideCapture:Float):Float;
  function calibrate(switchIds:Array<String>):Void;
  /** Attempt to release every held side, including after a controller fault. */
  function releaseAll():Void;
}

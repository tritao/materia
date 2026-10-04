package robotkit.runtime;

/** Motor-side holds for a homing owner. Holds use physical shaft coordinates. */
interface HomingSideControl {
  function hold(switchId:String):Void;
  /** Attempt to release every held side, including after a controller fault. */
  function releaseAll():Void;
}

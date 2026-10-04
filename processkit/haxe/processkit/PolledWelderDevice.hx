package processkit;

/** A nonblocking supply serviced exclusively by its device owner. */
interface PolledWelderDevice {
  function outputs():WelderOutputs;
  function feedback():WelderFeedback;
  function poll(now:Float):Void;
  function ready():Bool;
  function faultDetail():Null<String>;
  function safe():Void;
  function safeAcknowledged():Bool;
  function close():Void;
}

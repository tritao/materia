package processkit.screwdriving;

/** A nonblocking controller serviced independently of motion planning. */
interface PolledScrewdriverDevice extends ScrewdriverAcquisitionSource {
  function outputs():ScrewdriverOutputs;
  function feedback():ScrewdriverFeedback;
  function poll(now:Float):Void;
  function ready():Bool;
  function faultDetail():Null<String>;
  function safe():Void;
  function safeAcknowledged():Bool;
  function close():Void;
}

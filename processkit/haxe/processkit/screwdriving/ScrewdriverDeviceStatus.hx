package processkit.screwdriving;

/** Owner-serviced controller readiness, distinct from a tightening outcome. */
interface ScrewdriverDeviceStatus {
  function ready():Bool;
  function faultDetail():Null<String>;
}

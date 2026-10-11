package processkit.screwdriving;

/** Controller readiness and a priority stop that discards pending drive requests. */
interface ScrewdriverDeviceControl extends ScrewdriverDeviceStatus {
  function safe():Void;
}

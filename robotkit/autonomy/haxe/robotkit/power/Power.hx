package robotkit.power;

/** User-facing source of battery state for a robot. */
interface Power {
  function batteryState():Null<BatteryState>;
}

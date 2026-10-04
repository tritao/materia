package robotkit.runtime;

/** Optional scoped physical stop; false selects the driver's ordinary stop. */
interface HomingStopControl {
  function controlledStop():Bool;
}

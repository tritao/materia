package robotkit.runtime;

/** Optional acknowledgment contract for asynchronous physical side controls. */
interface HomingControlReadiness {
  /** True only after every issued operation was accepted; rejection/timeout throws. */
  function controlsReady():Bool;
}

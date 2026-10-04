package robotkit.runtime;

/** Immediate cleanup when a homing cycle will no longer poll its side controls. */
interface HomingCancellation {
  function cancelSquaring():Void;
}

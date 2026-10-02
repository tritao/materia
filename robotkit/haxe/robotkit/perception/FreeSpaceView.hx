package robotkit.perception;

/** The places a sensor sees through right now, in the localization reference frame. */
interface FreeSpaceView {
  /**
   * True when the sensor currently sees past the whole disk of `radiusMeters` about (x, y) with nothing
   * in it: whatever stood there has gone. False when it cannot tell (out of range, behind something).
   */
  function freeAt(x:Float, y:Float, radiusMeters:Float):Bool;
}

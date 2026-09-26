package motionkit;

/** A feed rate expressed in MotionKit's SI metres-per-second units. */
class Feed {
  public final value:Float;

  public function new(metresPerSecond:Float) {
    if (!Math.isFinite(metresPerSecond) || metresPerSecond <= 0.0)
      throw "Feed rate must be finite and positive";
    this.value = metresPerSecond;
  }

  public static function metresPerSecond(value:Float):Feed return new Feed(value);

  public static function mmPerSecond(value:Float):Feed return new Feed(value * 0.001);
}

package robotkit.protocol;

/** Requested RKF1 outbound family and maximum delivery rate. Zero is unlimited. */
@:wire
class StreamSubscription {
  @:id(1) public var family:String;
  @:id(2) public var maxRateHz:Float;

  public function new(family:String, ?maxRateHz:Float = 0.0) {
    this.family = family;
    this.maxRateHz = maxRateHz;
  }
}

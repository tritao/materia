package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingSegmentMsg {
  @:id(1) public var timeFromStartNs:haxe.Int64;
  @:id(2) public var durationNs:haxe.Int64;
  @:id(3) public var coefficients:Array<Array<Float>>;

  public function new() {}
}

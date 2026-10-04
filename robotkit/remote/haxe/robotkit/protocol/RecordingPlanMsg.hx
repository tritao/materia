package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingPlanMsg {
  @:id(1) public var planId:haxe.Int64;
  @:id(2) public var modelRevision:haxe.Int64;
  @:id(3) public var calibrationRevision:haxe.Int64;
  @:id(4) public var requiredCapabilities:Int;
  @:id(5) public var startPosition:Array<Float>;
  @:id(6) public var startVelocity:Array<Float>;
  @:id(7) public var startAcceleration:Array<Float>;
  @:id(8) public var positionTolerances:Array<Float>;
  @:id(9) public var velocityTolerances:Array<Float>;
  @:id(10) public var accelerationTolerances:Array<Float>;
  @:id(11) public var endsAtRest:Bool;
  @:id(12) public var jerkUnchecked:Bool;
  @:id(13) public var segments:Array<RecordingSegmentMsg>;
  @:id(14) public var events:Array<RecordingTimedEventMsg>;
  @:id(15) public var replaceAfterPlanId:haxe.Int64;
  @:id(16) public var replaceAfterTimeNs:haxe.Int64;

  @:optional @:id(17) public var controlAcceleration:Null<Array<Float>>;

  public function new() {}
}

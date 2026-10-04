package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingCommandMsg {
  @:id(1) public var kind:Int;
  @:id(2) public var robotId:String;
  @:id(3) public var targets:Array<RecordingJointTargetMsg>;
  @:id(4) public var expiryNs:haxe.Int64;
  @:id(7) public var plan:Null<RecordingPlanMsg>;

  public function new() {}
}

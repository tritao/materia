package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingWorldMsg {
  @:id(1) public var sequence:Int;
  @:id(2) public var topologyRevision:Int;
  @:id(3) public var sourceTimestampNs:haxe.Int64;
  @:id(4) public var receivedTimestampNs:haxe.Int64;
  @:id(5) public var robots:Array<RecordingSnapshotMsg>;

  public function new() {}
}

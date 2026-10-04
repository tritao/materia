package robotkit.protocol;
@:wire
class StreamSequenceMsg {
  @:id(1) public var streamId:String;
  @:id(2) public var kind:String;
  @:id(3) public var sequence:haxe.Int64;
  @:id(4) public var sourceClockId:String;
  public function new(?streamId:String = "", ?kind:String = "", ?sequence:haxe.Int64,
      ?sourceClockId:String = "unspecified") {
    this.streamId = streamId; this.kind = kind;
    this.sequence = sequence == null ? haxe.Int64.ofInt(0) : sequence;
    this.sourceClockId = sourceClockId;
  }
  public function value():robotkit.streams.StreamSequence
    return new robotkit.streams.StreamSequence(streamId,kind,sequence,sourceClockId);
  public static function values(source:Array<StreamSequenceMsg>):Array<robotkit.streams.StreamSequence>
    return [for (value in source) value.value()];
}

package robotkit.protocol;

import haxe.Int64;

/** Wire representation of a sensor frame; values stay transport-neutral. */
@:wire
class SensorFrameMsg {
  @:id(1) public var robotId:Int64;
  @:id(2) public var sensorId:String;
  @:id(3) public var kind:String;
  @:id(4) public var frameId:String;
  @:id(5) public var sequence:Int64;
  @:id(6) public var sourceTimestampNs:Int64;
  @:id(7) public var receivedTimestampNs:Int64;
  @:id(8) public var values:Array<Float>;
  @:id(9) public var linkId:String;
  @:id(10) public var mountPosition:Array<Float>;
  @:id(11) public var mountRotation:Array<Float>;

  public function new(?robotId:Int64 = null, ?sensorId:String = "",
      ?kind:String = "", ?frameId:String = "", ?sequence:Int64 = null,
      ?sourceTimestampNs:Int64 = null, ?receivedTimestampNs:Int64 = null,
      ?values:Array<Float> = null, ?linkId:String = "",
      ?mountPosition:Array<Float> = null, ?mountRotation:Array<Float> = null) {
    this.robotId = robotId == null ? Int64.ofInt(0) : robotId;
    this.sensorId = sensorId;
    this.kind = kind;
    this.frameId = frameId;
    this.sequence = sequence == null ? Int64.ofInt(0) : sequence;
    this.sourceTimestampNs = sourceTimestampNs == null ? Int64.ofInt(0) : sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0) : receivedTimestampNs;
    this.values = values == null ? [] : values.copy();
    this.linkId = linkId;
    this.mountPosition = mountPosition == null ? [0.0, 0.0, 0.0] : mountPosition.copy();
    this.mountRotation = mountRotation == null ? [0.0, 0.0, 0.0, 1.0] : mountRotation.copy();
  }
}

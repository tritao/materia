package tests;

@:wire
class TestRecordingPayload {
  @:id(1) public var robotId:String;
  @:id(2) public var value:Int;
  public function new() {}
}

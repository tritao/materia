package robotkit.protocol;

import robotkit.streams.ImageDetection;

@:wire
class ImageDetectionMsg {
  @:id(1) public var label:String;
  @:id(2) public var score:Float;
  @:id(3) public var x:Float;
  @:id(4) public var y:Float;
  @:id(5) public var width:Float;
  @:id(6) public var height:Float;

  public function new(?label:String = "", ?score:Float = 0, ?x:Float = 0,
      ?y:Float = 0, ?width:Float = 0, ?height:Float = 0) {
    this.label = label; this.score = score; this.x = x;
    this.y = y; this.width = width; this.height = height;
  }

  public static function fromDetection(value:ImageDetection):ImageDetectionMsg
    return new ImageDetectionMsg(value.label, value.score, value.x, value.y,
      value.width, value.height);

  public function toDetection():ImageDetection
    return new ImageDetection(label, score, x, y, width, height);
}

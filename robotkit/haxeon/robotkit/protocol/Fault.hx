package robotkit.protocol;

@:wire
class Fault {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var code:Int;
  @:id(3) public var message:String;
  @:id(4) public var fatal:Bool;

  public function new(?robotId:haxe.Int64 = null, ?code:Int = 0,
      ?message:String = "", ?fatal:Bool = false) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.code = code;
    this.message = message;
    this.fatal = fatal;
  }
}

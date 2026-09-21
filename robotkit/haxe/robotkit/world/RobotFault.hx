package robotkit.world;

/** Transport-independent fault reported by a logical robot. */
class RobotFault {
  public final id:RobotId;
  public final code:Int;
  public final message:String;
  public final fatal:Bool;

  public function new(id:RobotId, code:Int, message:String, fatal:Bool) {
    this.id = id;
    this.code = code;
    this.message = message;
    this.fatal = fatal;
  }
}

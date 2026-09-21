package robotkit.protocol;

/** Logical reference to a RobotFrame attachment. */
@:wire
class BufferRef {
  @:id(1) public var attachment:Int;
  @:id(2) public var offset:Int;
  @:id(3) public var length:Int;

  public function new(?attachment:Int = 0, ?offset:Int = 0, ?length:Int = 0) {
    this.attachment = attachment;
    this.offset = offset;
    this.length = length;
  }
}

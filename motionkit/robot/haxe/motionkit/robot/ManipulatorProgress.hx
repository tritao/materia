package motionkit.robot;

/** Position in the authored program and current path. */
class ManipulatorProgress {
  public final block:Int;
  public final op:Int;
  public final pathDistance:Float;
  public function new(block:Int, op:Int, pathDistance:Float) {
    this.block = block; this.op = op; this.pathDistance = pathDistance;
  }
}

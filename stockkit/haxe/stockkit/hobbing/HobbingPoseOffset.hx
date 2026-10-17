package stockkit.hobbing;

/** Small rigid translations in mm. Work offsets rotate with C; tool offsets
    rotate with B. Separate frames prevent radial hob error becoming an X-only wobble. */
class HobbingPoseOffset {
  public var workX:Float=0;public var workY:Float=0;public var workZ:Float=0;
  public var toolX:Float=0;public var toolY:Float=0;public var toolZ:Float=0;
  public function new() {}
  public function copy():HobbingPoseOffset {
    var out=new HobbingPoseOffset();out.workX=workX;out.workY=workY;out.workZ=workZ;
    out.toolX=toolX;out.toolY=toolY;out.toolZ=toolZ;return out;
  }
}

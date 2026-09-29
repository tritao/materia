package visionkit;

import VisionKitNative;

/** B's pose in A, with metres and x,y,z,w quaternion order. */
class Pose3 {
  public final x:Float; public final y:Float; public final z:Float;
  public final qx:Float; public final qy:Float; public final qz:Float; public final qw:Float;
  public function new(x:Float, y:Float, z:Float,
      qx:Float, qy:Float, qz:Float, qw:Float) {
    this.x=x; this.y=y; this.z=z;
    this.qx=qx; this.qy=qy; this.qz=qz; this.qw=qw;
  }
  public static function fromNative(n:vk_pose3):Pose3
    return new Pose3(n.get_x(), n.get_y(), n.get_z(),
      n.get_qx(), n.get_qy(), n.get_qz(), n.get_qw());
}

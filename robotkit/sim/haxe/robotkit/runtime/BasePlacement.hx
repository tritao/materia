package robotkit.runtime;

import robotkit.mobile.Pose2;

/** Planar base placement shared by the wheeled drive plants. */
class BasePlacement {
  /**
   * Jumps a robot's base to a planar pose for the next tick at `height`,
   * keeping its roll and pitch relative to its heading, as the native drive
   * plants do while they drive. The jump carries the base's velocity and does
   * not read as motion; see Simulation.placeRobotBase.
   */
  public static function place(simulation:Simulation, robotIndex:Int, pose:Pose2,
      height:Float):Void {
    var q = simulation.robotPose(robotIndex).rotation;
    // Tilt = Rz(-heading) * rotation: the current rotation with its heading
    // (the body x axis projected on the floor) removed.
    var heading = Math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
    var tilt = multiply(yawRotation(-heading), q);
    simulation.placeRobotBase(robotIndex, [pose.x, pose.y, height],
      multiply(yawRotation(pose.yaw), tilt));
  }

  static function yawRotation(yaw:Float):Array<Float> {
    var half = yaw * 0.5;
    return [0.0, 0.0, Math.sin(half), Math.cos(half)];
  }

  // Hamilton product of xyzw quaternions, normalized.
  static function multiply(a:Array<Float>, b:Array<Float>):Array<Float> {
    var x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    var y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    var z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    var w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    var norm = Math.sqrt(x * x + y * y + z * z + w * w);
    return [x / norm, y / norm, z / norm, w / norm];
  }
}

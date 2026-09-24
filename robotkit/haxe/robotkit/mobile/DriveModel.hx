package robotkit.mobile;

import robotkit.world.JointTarget;

/** Kinematic mapping from a planar body twist to one atomic joint target batch. */
interface DriveModel {
  function constrain(twist:Twist2):Twist2;
  function targets(twist:Twist2):Array<JointTarget>;
  function createOdometry():Null<DifferentialOdometry>;
}

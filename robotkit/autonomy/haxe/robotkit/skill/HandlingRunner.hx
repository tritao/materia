package robotkit.skill;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/**
 * Arm motion for picking a part up and setting it down. RobotKit names the motion; MotionKit
 * implements it (`motionkit.robot.HandlingPlanRunner`), so RobotKit itself stays free of MotionKit,
 * as with `SurfacePlanRunner`.
 */
interface HandlingRunner {
  /**
   * Brings the tool's contact to `contact`, in the arm's base frame, straight down from above it;
   * switches the tool's hold channel to `hold` there; rises again and returns the arm to its home
   * pose. An explicit base-frame orientation fixes the tool heading; otherwise its
   * home Z direction is constrained and spin is free.
   */
  function run(contact:Vec3, hold:Bool, ?orientation:Quat):Void;
  function update(dtSeconds:Float):Void;
  function abort():Void;
  function running():Bool;
  function completed():Bool;
  function failure():Null<String>;
}

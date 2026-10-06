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
   * Brings the tool's contact to `contact`, in the arm's base frame, from `approach`;
   * switches the tool's hold channel to `hold` there and rises again. Without an explicit
   * orientation it returns to the home pose; with one it stays at the raised pose, preserving a
   * held part for the next operation. A missing approach uses the conventional base +Z direction.
   */
  function run(contact:Vec3, hold:Bool, ?orientation:Quat, ?approach:Vec3):Void;
  function update(dtSeconds:Float):Void;
  function abort():Void;
  function running():Bool;
  function completed():Bool;
  function failure():Null<String>;
}

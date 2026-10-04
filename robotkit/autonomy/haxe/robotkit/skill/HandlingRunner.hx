package robotkit.skill;

import robotkit.spatial.Vec3;

/**
 * Arm motion for picking a part up and setting it down. RobotKit names the motion; MotionKit
 * implements it (`motionkit.robot.HandlingPlanRunner`), so RobotKit itself stays free of MotionKit,
 * as with `SurfacePlanRunner`.
 */
interface HandlingRunner {
  /**
   * Brings the tool's contact to `contact`, in the arm's base frame, straight down from above it;
   * switches the tool's hold channel to `hold` there; rises again and returns the arm to its home
   * pose. The tool keeps the orientation it has at home, as a suction cup facing down does.
   */
  function run(contact:Vec3, hold:Bool):Void;
  function update(dtSeconds:Float):Void;
  function abort():Void;
  function running():Bool;
  function completed():Bool;
  function failure():Null<String>;
}

package robotkit.skill;

import robotkit.tool.WeldSensor.WeldReading;

/**
 * Arm motion and process control for welding one seam. RobotKit names the work; ProcessKit implements it
 * (`processkit.WeldingPlanRunner`, a MotionKit program driven by a process run), so RobotKit itself stays free of
 * both, as with `HandlingRunner`. It works through the robot alone: the torch's channels and the motion, and the
 * welder's reading, which the caller hands in each tick from whatever reports it.
 */
interface WeldRunner {
  /** Starts welding `plan`, in the arm's base frame. */
  function run(plan:WeldPlan):Void;
  /** Advances `dtSeconds` with the welder's latest `reading`. */
  function update(dtSeconds:Float, reading:WeldReading):Void;
  function abort():Void;
  function running():Bool;
  /** The seam is welded and the torch is clear of it. */
  function completed():Bool;
  function failure():Null<String>;
  /** How many times the arc was lost and the weld restarted. */
  function restarts():Int;
}

package motionkit;

import motionkit.axis.MotionSystemBlueprint;
import motionkit.planner.TrajectoryPlanner;
import robotkit.world.Robot;

/** Root-package facade preserving the concise public MotionKit API. */
class MotionSystem extends motionkit.axis.MotionSystem {
  public static function fromBlueprint(robot:Robot, blueprint:MotionSystemBlueprint):MotionSystem
    return new MotionSystem(robot, blueprint);

  public function new(robot:Robot, blueprint:MotionSystemBlueprint,
      ?planner:TrajectoryPlanner)
    super(robot, blueprint, planner);
}

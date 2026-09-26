package motionkit.planner;

import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.MotionLimits;

/** Planning boundary shared by simulation and future hardware backends. */
interface TrajectoryPlanner {
  function plan(startPositions:Array<Float>, goalPositions:Array<Float>,
      limits:MotionLimits):JointTrajectory;
}

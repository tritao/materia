package motionkit.robot;

import motionkit.kinematics.PathRequest;
import motionkit.path.PosePath;
import motionkit.planner.JointPathSamples;

/** Select and refine one path; timing consumes the returned joint curve. */
interface JointPathPlanner {
  function withSolver(solver:motionkit.kinematics.KinematicsSolver):JointPathPlanner;
  function plan(path:PosePath,request:PathRequest):JointPathSamples;
}

package motionkit.robot;

import motionkit.kinematics.PathRequest;
import motionkit.path.PosePath;
import motionkit.planner.JointPathSamples;

/** Select and refine one path; timing consumes the returned joint curve. */
interface JointPathPlanner {
  function withConfiguration(configuration:motionkit.kinematics.SixAxisConfiguration):JointPathPlanner;
  function withSolver(solver:motionkit.kinematics.KinematicsSolver):JointPathPlanner;
  function allowsFreeStart():Bool;
  function retreatTarget():Null<Array<Float>>;
  function checkPathClearance(path:JointPathSamples,tolerance:Float):Bool;
  function checkMotion(trajectory:motionkit.trajectory.Trajectory):Null<robotkit.manipulation.ClearanceViolation>;
  /**
   * The conservative clearance check of a timed motion (`TrajectoryClearanceProof`), or null
   * without a clearance world. `velocity` bounds each joint's speed over the motion.
   */
  function proveMotion(trajectory:motionkit.trajectory.Trajectory, velocity:Array<Float>, events:Array<ClearanceEvent>,
    depthLimit:Int):Null<TrajectoryClearanceProof>;
  /** The clearance world the planner checks with, or null. */
  function clearanceWorld():Null<robotkit.manipulation.ClearanceWorld>;
  function plan(path:PosePath,request:PathRequest,?pinStart:Bool,
    ?entryCheck:(Array<Float>,Array<Float>)->Null<robotkit.manipulation.ClearanceViolation>,
    ?exitCheck:Array<Float>->Null<robotkit.manipulation.ClearanceViolation>):JointPathSamples;
}

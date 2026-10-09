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
  function plan(path:PosePath,request:PathRequest,?pinStart:Bool,
    ?entryCheck:(Array<Float>,Array<Float>)->Null<robotkit.manipulation.ClearanceViolation>,
    ?exitCheck:Array<Float>->Null<robotkit.manipulation.ClearanceViolation>):JointPathSamples;
}

package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;

/**
 * Names the motion a group has left once its tool pose is fixed
 * (KINEMATICS.md KK-D19), so `RedundancyResolver` can choose it along a
 * whole path: the swivel of a 7-axis arm, the values of a cell's external
 * axes, later a mobile base's pose.
 */
interface RedundancyParameterization {
  /** How many values name the redundancy. */
  function dimension():Int;
  /** The values at `q`, or null where they are undefined (e.g. a straight elbow). */
  function valuesAt(q:Array<Float>):Null<Array<Float>>;
  /** IK for the tool target with the redundancy held at `values`, seeded by `seed`; null if it does not solve. */
  function solveAt(target:Pose3, seed:Array<Float>, values:Array<Float>, tolerance:IkTolerance):Null<Array<Float>>;
  /** IK keeping the redundancy as near the seed's as the target allows (where `solveAt` is blocked, e.g. by a limit). */
  function solveNear(target:Pose3, seed:Array<Float>, tolerance:IkTolerance):Null<Array<Float>>;
  /** How far each value may move per metre of path between samples: the resolver's lattice resolution. */
  function ratesPerMetre():Array<Float>;
  /** True for a value that wraps (an angle), so smoothing never averages across the ±π seam. */
  function periodic(index:Int):Bool;
  /** Factor on each group DOF's motion cost in the path search (below 1 makes a DOF the cheaper one to move). */
  function costFactors():Array<Float>;
}

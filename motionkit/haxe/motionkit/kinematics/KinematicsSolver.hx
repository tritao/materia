package motionkit.kinematics;

/** Transport-neutral forward, inverse, differential and path kinematics contract. */
interface KinematicsSolver {
  function jointCount():Int;
  /**
   * This solver for another thread: the same kinematics and settings, sharing nothing that either
   * changes, so a planner can solve while its caller keeps using the original. A solver holding
   * no mutable state returns itself.
   */
  function fork():KinematicsSolver;
  function forward(q:Array<Float>):Pose3;
  function solvePose(target:Pose3, seed:Array<Float>,
    tolerance:IkTolerance):Null<Array<Float>>;
  function sampleCandidates(target:Pose3, maxCount:Int,
    tolerance:IkTolerance):Array<Array<Float>>;
  /**
   * Joint rates producing `twist` at `q`. A redundant solver has a family of them; given
   * `redundancyRate` (d/ds of its redundancy values, e.g. `PathSolution.redundancyRates` at the sample)
   * it returns the one moving its redundancy (a swivel, external axes) at exactly that rate; without
   * one, the smallest. Solvers with no redundancy ignore it.
   */
  function solveDifferential(q:Array<Float>, twist:Twist6, ?redundancyRate:Array<Float>):Null<Array<Float>>;
  /**
   * One configuration per sample of a path (see `PathRequest`): each solver
   * searches the way that suits it (an analytic arm across its branches, a
   * redundant group across its redundancy, a plain one point by point).
   * Null for a sample that cannot be reached; a search that finds no route
   * may throw its diagnostic instead.
   */
  function solvePath(request:PathRequest):Array<Null<Array<Float>>>;
}

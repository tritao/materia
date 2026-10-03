package motionkit.kinematics;

/** A solver that chooses its redundancy along a path and reports how that choice changes (see `PathSolution`). */
interface RedundantPathSolver extends KinematicsSolver {
  function solvePathWithRates(request:PathRequest):PathSolution;
}

package robotkit.process;

import robotkit.spatial.Transform3;
import robotkit.manipulation.Manipulator;

/**
 * Drives a `CartesianTrajectory` through a `Manipulator`'s TCP-target IK,
 * seeded by the previous sample's solution, and rejects joint-space jumps
 * above `maxJointStep`. `base_T_work` brings the trajectory's own frame
 * (`toolpath.frameId`) into the manipulator chain's base frame; callers
 * resolve that transform (e.g. via `FrameTree3.lookup`) before calling.
 * Failure is an explicit result value, never an exception, for an
 * unreachable point or a joint discontinuity.
 */
class ToolpathExecutor {
  public static function execute(manipulator:Manipulator, trajectory:CartesianTrajectory,
      base_T_work:Transform3, seed:Array<Float>, ?maxJointStep:Float = 0.5,
      ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3,
      ?maxIterations:Int = 100, ?damping:Float = 0.02):ToolpathExecutionResult {
    if (manipulator == null || trajectory == null || base_T_work == null)
      throw "Toolpath execution requires a manipulator, trajectory, and base_T_work transform";
    var steps:Array<ToolpathExecutionStep> = [];
    var previousQ:Null<Array<Float>> = seed == null ? null : seed.copy();
    for (index in 0...trajectory.samples.length) {
      var sample = trajectory.samples[index];
      var target = base_T_work.compose(sample.work_T_tcp);
      var ik = manipulator.solveIkForTcp(target, previousQ, positionTolerance,
        orientationTolerance, maxIterations, damping);
      if (!ik.converged)
        return new ToolpathExecutionResult(false, steps, ToolpathExecutionFailure.Unreachable(index, ik));
      if (previousQ != null) {
        for (joint in 0...ik.q.length) {
          var delta = Math.abs(ik.q[joint] - previousQ[joint]);
          if (delta > maxJointStep)
            return new ToolpathExecutionResult(false, steps,
              ToolpathExecutionFailure.Discontinuity(index, joint, delta));
        }
      }
      steps.push(new ToolpathExecutionStep(sample.time, manipulator.toJointTargets(ik.q), ik.q, sample.processOn));
      previousQ = ik.q;
    }
    return new ToolpathExecutionResult(true, steps);
  }
}

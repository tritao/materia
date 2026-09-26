package robotkit.process;

import robotkit.manipulation.IKResult;

/** Explicit failure modes for `ToolpathExecutor.execute`; never thrown. */
enum ToolpathExecutionFailure {
  /** IK did not converge for the sample at `sampleIndex`. */
  Unreachable(sampleIndex:Int, ik:IKResult);
  /** Joint `jointIndex` moved more than the allowed step between consecutive samples. */
  Discontinuity(sampleIndex:Int, jointIndex:Int, delta:Float);
}

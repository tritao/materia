package motionkit.planner;

interface PathTimingBackend {
  /** Times q(s), lowers it, and retains the same distance-to-time law. */
  function time(path:JointPathSamples, limits:PathTimingLimits):TimedPath;
}

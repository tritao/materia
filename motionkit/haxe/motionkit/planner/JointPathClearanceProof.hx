package motionkit.planner;

/** Immutable evidence for a particular curve and a bounded lowering error. */
interface JointPathClearanceProof {
  function covers(path:JointPathSamples,tolerance:Float):Bool;
}

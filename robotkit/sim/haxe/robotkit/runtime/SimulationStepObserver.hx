package robotkit.runtime;

import haxe.Int64;

/** Called after an explicit Simulation.step or its owning SimSession.step
 * with a monotonic logical source time. */
interface SimulationStepObserver {
  function afterSimulationStep(sourceTimestampNs:Int64):Void;
}

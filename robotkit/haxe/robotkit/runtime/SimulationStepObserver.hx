package robotkit.runtime;

import haxe.Int64;

/** Called after an explicit Simulation.step with its logical source time. */
interface SimulationStepObserver {
  function afterSimulationStep(sourceTimestampNs:Int64):Void;
}

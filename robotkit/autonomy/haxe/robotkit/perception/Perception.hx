package robotkit.perception;



import robotkit.core.SensorFrame;

/** Converts transport-neutral sensor observations into semantic values. */
interface Perception {
  function observe(frames:Array<SensorFrame>):PerceptionSnapshot;
}

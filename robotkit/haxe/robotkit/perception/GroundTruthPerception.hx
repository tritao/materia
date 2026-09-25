package robotkit.perception;

import robotkit.world.SensorFrame;

/** Supplies scene truth through the same API used by sensor-derived perception. */
class GroundTruthPerception implements Perception {
  final snapshotProvider:Void -> PerceptionSnapshot;

  /**
   * The provider should read the simulation or test scene at observation time.
   * Sensor frames are intentionally ignored; all returned values must already
   * carry the scene's reference frame and timestamp metadata.
   */
  public function new(snapshotProvider:Void -> PerceptionSnapshot) {
    if (snapshotProvider == null)
      throw "Ground-truth perception requires a scene snapshot provider";
    this.snapshotProvider = snapshotProvider;
  }

  public function observe(frames:Array<SensorFrame>):PerceptionSnapshot {
    if (frames == null) throw "Ground-truth perception requires a sensor frame batch";
    var value = snapshotProvider();
    if (value == null) throw "Ground-truth scene provider returned no snapshot";
    return new PerceptionSnapshot(value.detections(), value.obstacles(),
      value.pallets(), value.dockingTargets());
  }
}

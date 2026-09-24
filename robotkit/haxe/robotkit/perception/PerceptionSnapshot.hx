package robotkit.perception;

/** Immutable semantic values produced from a batch of sensor frames. */
class PerceptionSnapshot {
  final detectionValues:Array<Detection>;
  final obstacleValues:Array<Obstacle>;
  final palletValues:Array<Pallet>;
  final dockingValues:Array<DockingTarget>;

  public function new(?detections:Array<Detection>, ?obstacles:Array<Obstacle>,
      ?pallets:Array<Pallet>, ?dockingTargets:Array<DockingTarget>) {
    detectionValues = detections == null ? [] : detections.copy();
    obstacleValues = obstacles == null ? [] : obstacles.copy();
    palletValues = pallets == null ? [] : pallets.copy();
    dockingValues = dockingTargets == null ? [] : dockingTargets.copy();
    for (value in detectionValues) if (value == null) throw "Perception detections cannot be null";
    for (value in obstacleValues) if (value == null) throw "Perception obstacles cannot be null";
    for (value in palletValues) if (value == null) throw "Perception pallets cannot be null";
    for (value in dockingValues) if (value == null) throw "Perception docking targets cannot be null";
  }

  public function detections():Array<Detection> return detectionValues.copy();
  public function obstacles():Array<Obstacle> return obstacleValues.copy();
  public function pallets():Array<Pallet> return palletValues.copy();
  public function dockingTargets():Array<DockingTarget> return dockingValues.copy();
}

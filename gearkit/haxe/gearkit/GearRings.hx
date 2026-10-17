package gearkit;

/** Read-only metrology boundary, mm/radians. Keeps GearKit independent of StockKit.
    Boundaries are paired material interval endpoints, sorted in [0,2pi]. */
interface GearRings {
  public function radii():Array<Float>;
  public function axialPositions():Array<Float>;
  public function boundaries(radial:Int, axial:Int):Array<Float>;
}

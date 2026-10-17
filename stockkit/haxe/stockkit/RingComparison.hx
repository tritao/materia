package stockkit;

/** Per-circle angular errors (radians), axial-major, radial index fastest.
    Arc lengths and cell-volume estimates use mm and cubic mm. This is geometric
    leftover/gouge, not an ISO gear grade; HB4 supplies metrology evaluation. */
class RingComparison {
  public final grid:RingGrid;
  public final leftover:Array<Float>;
  public final gouge:Array<Float>;
  public function new(grid:RingGrid, leftover:Array<Float>, gouge:Array<Float>) {
    this.grid = grid; this.leftover = leftover; this.gouge = gouge;
  }
  public function leftoverVolume():Float return volume(leftover);
  public function gougeVolume():Float return volume(gouge);
  public function leftoverArc(radialIndex:Int, axialIndex:Int):Float
    return at(leftover, radialIndex, axialIndex) * grid.radius(radialIndex);
  public function gougeArc(radialIndex:Int, axialIndex:Int):Float
    return at(gouge, radialIndex, axialIndex) * grid.radius(radialIndex);
  function at(values:Array<Float>, r:Int, z:Int):Float {
    grid.radius(r); grid.z(z);
    return values[z * grid.radialCount + r];
  }
  function volume(values:Array<Float>):Float {
    var total = 0.0;
    for (z in 0...grid.axialCount) for (r in 0...grid.radialCount)
      total += values[z * grid.radialCount + r] * grid.sectorVolume(r);
    return total;
  }
}

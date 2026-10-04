package robotkit.navigation;

/** Integer address of one cell in an OccupancyGrid2. */
class GridCell2 {
  public final x:Int;
  public final y:Int;

  public function new(x:Int, y:Int) {
    this.x = x;
    this.y = y;
  }
}

package robotkit.work;

/**
 * A regular grid over a `WorkSurface`'s boundary bounding box (mirroring
 * `CoverageMap`'s grid), accumulating the mean signed out-of-plane deviation
 * (surface-vs-design, meters) reported for each cell. A caller adds samples
 * already expressed in the surface's own local plane (`x, y` in-plane,
 * `deviation` the signed distance from the design/registered plane at that
 * `x, y` — e.g. a registered point cloud's residual `z` after removing the
 * rigid registration correction), so a smooth bow shows up as a spatial
 * pattern across cells rather than a single scalar.
 */
class DeviationMap {
  public final surface:WorkSurface;
  public final cellSize:Float;

  final columns:Int;
  final rowsCount:Int;
  final originX:Float;
  final originY:Float;
  final sum:Array<Float>;
  final count:Array<Int>;

  public function new(surface:WorkSurface, cellSize:Float) {
    if (surface == null) throw "Deviation map requires a work surface";
    if (!Math.isFinite(cellSize) || cellSize <= 0.0) throw "Deviation map cell size must be positive and finite";
    this.surface = surface;
    this.cellSize = cellSize;
    var bounds = surface.boundary.bounds();
    this.originX = bounds.minX;
    this.originY = bounds.minY;
    this.columns = Math.ceil((bounds.maxX - bounds.minX) / cellSize);
    this.rowsCount = Math.ceil((bounds.maxY - bounds.minY) / cellSize);
    var total = columns * rowsCount;
    sum = [for (_ in 0...total) 0.0];
    count = [for (_ in 0...total) 0];
  }

  /** Accumulates one `(x, y) -> deviation` sample; ignored if outside the grid. */
  public function addSample(x:Float, y:Float, deviation:Float):Void {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(deviation))
      throw "Deviation map sample must be finite";
    var index = cellIndex(x, y);
    if (index < 0) return;
    sum[index] += deviation;
    count[index]++;
  }

  /** Convenience: adds every point's `(x, y)` as the location and `z` as the deviation. */
  public function addPoints(points:Array<robotkit.spatial.Vec3>):Void {
    if (points == null) throw "Deviation map requires a points array";
    for (point in points) addSample(point.x, point.y, point.z);
  }

  public function hasSample(x:Float, y:Float):Bool {
    var index = cellIndex(x, y);
    return index >= 0 && count[index] > 0;
  }

  /** Mean accumulated deviation of the cell containing `(x, y)`; 0.0 if outside the grid or unsampled. */
  public function deviationAt(x:Float, y:Float):Float {
    var index = cellIndex(x, y);
    if (index < 0 || count[index] == 0) return 0.0;
    return sum[index] / count[index];
  }

  public function maxAbsDeviation():Float {
    var maxValue = 0.0;
    for (i in 0...sum.length) if (count[i] > 0) {
      var value = Math.abs(sum[i] / count[i]);
      if (value > maxValue) maxValue = value;
    }
    return maxValue;
  }

  public function meanAbsDeviation():Float {
    var total = 0.0;
    var sampled = 0;
    for (i in 0...sum.length) if (count[i] > 0) {
      total += Math.abs(sum[i] / count[i]);
      sampled++;
    }
    return sampled == 0 ? 0.0 : total / sampled;
  }

  function cellIndex(x:Float, y:Float):Int {
    var col = Math.floor((x - originX) / cellSize);
    var row = Math.floor((y - originY) / cellSize);
    if (col < 0 || row < 0 || col >= columns || row >= rowsCount) return -1;
    return row * columns + col;
  }
}

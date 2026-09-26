package robotkit.work;

/**
 * A regular grid over a `WorkSurface`'s boundary bounding box, tracking
 * which cells a process-on TCP footprint has covered. Each cell is
 * classified once at construction, by its center, as `allowed` (inside the
 * boundary and no exclusion), `excluded` (inside the boundary and inside an
 * exclusion), or neither (outside the boundary entirely).
 * `coverageFraction` reports allowed coverage; `exclusionCoverageFraction`
 * reports whether the process ever touched an excluded cell, independent of
 * simply falling outside the boundary.
 */
class CoverageMap {
  public final surface:WorkSurface;
  public final cellSize:Float;

  final columns:Int;
  final rowsCount:Int;
  final originX:Float;
  final originY:Float;
  final allowedFlag:Array<Bool>;
  final excludedFlag:Array<Bool>;
  final covered:Array<Bool>;

  public function new(surface:WorkSurface, cellSize:Float) {
    if (surface == null) throw "Coverage map requires a work surface";
    if (!Math.isFinite(cellSize) || cellSize <= 0.0) throw "Coverage map cell size must be positive and finite";
    this.surface = surface;
    this.cellSize = cellSize;
    var bounds = surface.boundary.bounds();
    this.originX = bounds.minX;
    this.originY = bounds.minY;
    this.columns = Math.ceil((bounds.maxX - bounds.minX) / cellSize);
    this.rowsCount = Math.ceil((bounds.maxY - bounds.minY) / cellSize);
    var total = columns * rowsCount;
    allowedFlag = [for (_ in 0...total) false];
    excludedFlag = [for (_ in 0...total) false];
    covered = [for (_ in 0...total) false];
    for (row in 0...rowsCount) for (col in 0...columns) {
      var center = cellCenter(col, row);
      var index = row * columns + col;
      if (!surface.boundary.contains(center)) continue;
      var excluded = false;
      for (exclusion in surface.exclusions) if (exclusion.contains(center)) {
        excluded = true;
        break;
      }
      if (excluded) excludedFlag[index] = true; else allowedFlag[index] = true;
    }
  }

  public function cellCenter(col:Int, row:Int):Point2
    return new Point2(originX + (col + 0.5) * cellSize, originY + (row + 0.5) * cellSize);

  /** Marks every cell whose center lies within `radius` of `center` as covered, regardless of classification. */
  public function markFootprint(center:Point2, radius:Float):Void {
    if (center == null) throw "Coverage footprint requires a center point";
    if (!Math.isFinite(radius) || radius < 0.0) throw "Coverage footprint radius must be finite and non-negative";
    var minCol = Math.floor((center.x - radius - originX) / cellSize);
    var maxCol = Math.floor((center.x + radius - originX) / cellSize);
    var minRow = Math.floor((center.y - radius - originY) / cellSize);
    var maxRow = Math.floor((center.y + radius - originY) / cellSize);
    if (minCol < 0) minCol = 0;
    if (minRow < 0) minRow = 0;
    if (maxCol >= columns) maxCol = columns - 1;
    if (maxRow >= rowsCount) maxRow = rowsCount - 1;
    var row = minRow;
    while (row <= maxRow) {
      var col = minCol;
      while (col <= maxCol) {
        var cell = cellCenter(col, row);
        var dx = cell.x - center.x, dy = cell.y - center.y;
        if (dx * dx + dy * dy <= radius * radius) covered[row * columns + col] = true;
        col++;
      }
      row++;
    }
  }

  /** Marks the footprint at evenly-spaced samples (at most `stepSize` apart) along a straight move. */
  public function markSweep(from:Point2, to:Point2, radius:Float, stepSize:Float):Void {
    if (from == null || to == null) throw "Coverage sweep requires endpoints";
    if (!Math.isFinite(stepSize) || stepSize <= 0.0) throw "Coverage sweep step size must be positive and finite";
    var dx = to.x - from.x, dy = to.y - from.y;
    var distance = Math.sqrt(dx * dx + dy * dy);
    var steps = distance <= stepSize ? 1 : Math.ceil(distance / stepSize);
    var i = 0;
    while (i <= steps) {
      var t = steps == 0 ? 0.0 : i / steps;
      markFootprint(new Point2(from.x + dx * t, from.y + dy * t), radius);
      i++;
    }
  }

  public function coverageFraction():Float {
    var allowedCount = 0, coveredCount = 0;
    for (i in 0...allowedFlag.length) if (allowedFlag[i]) {
      allowedCount++;
      if (covered[i]) coveredCount++;
    }
    return allowedCount == 0 ? 0.0 : coveredCount / allowedCount;
  }

  /** Fraction of excluded cells the process footprint nonetheless touched; should be zero. */
  public function exclusionCoverageFraction():Float {
    var excludedCount = 0, coveredCount = 0;
    for (i in 0...excludedFlag.length) if (excludedFlag[i]) {
      excludedCount++;
      if (covered[i]) coveredCount++;
    }
    return excludedCount == 0 ? 0.0 : coveredCount / excludedCount;
  }

  /** Excluded cell centers a footprint nonetheless touched; should be empty. */
  public function excludedCellsTouched():Array<Point2> {
    var result:Array<Point2> = [];
    for (row in 0...rowsCount) for (col in 0...columns) {
      var index = row * columns + col;
      if (excludedFlag[index] && covered[index]) result.push(cellCenter(col, row));
    }
    return result;
  }

  /** Allowed cell centers that remain uncovered. */
  public function uncoveredCells():Array<Point2> {
    var result:Array<Point2> = [];
    for (row in 0...rowsCount) for (col in 0...columns) {
      var index = row * columns + col;
      if (allowedFlag[index] && !covered[index]) result.push(cellCenter(col, row));
    }
    return result;
  }
}

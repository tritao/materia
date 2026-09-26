package robotkit.work;

/**
 * A regular grid of terrain elevation samples in a named frame (mirroring
 * `CoverageMap`/`DeviationMap`'s grid conventions), used by the excavator
 * milestone. Unlike `CoverageMap`, samples live at grid *vertices*
 * (`columns x rows` points spaced `cellSize` apart, starting at
 * `(originX, originY)`), which is what makes `bilinearSample` and
 * `volumeBetween`'s per-cell trapezoidal average well defined. Elevation is
 * mutable state (like `CoverageMap.covered`), not an immutable value type:
 * it models terrain that a `BucketSweep` physically lowers over time.
 */
class HeightMap {
  public final frameId:String;
  public final originX:Float;
  public final originY:Float;
  public final cellSize:Float;
  public final columns:Int;
  public final rows:Int;

  final elevation:Array<Float>;

  public function new(frameId:String, originX:Float, originY:Float, cellSize:Float,
      columns:Int, rows:Int, ?elevation:Array<Float>) {
    if (frameId == null || frameId.length == 0) throw "Height map requires a non-empty frame id";
    if (!Math.isFinite(originX) || !Math.isFinite(originY)) throw "Height map origin must be finite";
    if (!Math.isFinite(cellSize) || cellSize <= 0.0) throw "Height map cell size must be positive and finite";
    if (columns < 2 || rows < 2) throw "Height map requires at least a 2x2 grid of vertices";
    this.frameId = frameId;
    this.originX = originX;
    this.originY = originY;
    this.cellSize = cellSize;
    this.columns = columns;
    this.rows = rows;
    if (elevation == null) {
      this.elevation = [for (_ in 0...(columns * rows)) 0.0];
    } else {
      if (elevation.length != columns * rows)
        throw 'Height map elevation array must have exactly ${columns * rows} entries, got ${elevation.length}';
      this.elevation = elevation.copy();
    }
  }

  public function worldX(col:Int):Float return originX + col * cellSize;
  public function worldY(row:Int):Float return originY + row * cellSize;

  public function elevationAt(col:Int, row:Int):Float {
    if (col < 0 || col >= columns || row < 0 || row >= rows)
      throw 'Height map vertex ($col, $row) is out of bounds';
    return elevation[row * columns + col];
  }

  /** Lowers a vertex to `z` if it is currently higher; no-op (never raises terrain) otherwise. */
  public function lowerTo(col:Int, row:Int, z:Float):Void {
    if (col < 0 || col >= columns || row < 0 || row >= rows)
      throw 'Height map vertex ($col, $row) is out of bounds';
    if (!Math.isFinite(z)) throw "Height map elevation must be finite";
    var index = row * columns + col;
    if (z < elevation[index]) elevation[index] = z;
  }

  /** Sets a vertex's elevation directly (raises or lowers); used to author fixtures. */
  public function setElevation(col:Int, row:Int, z:Float):Void {
    if (col < 0 || col >= columns || row < 0 || row >= rows)
      throw 'Height map vertex ($col, $row) is out of bounds';
    if (!Math.isFinite(z)) throw "Height map elevation must be finite";
    elevation[row * columns + col] = z;
  }

  /** Returns an independent mutable copy with the same grid and elevations. */
  public function copy():HeightMap
    return new HeightMap(frameId, originX, originY, cellSize, columns, rows, elevation);

  /** Bilinearly-interpolated elevation at an arbitrary in-plane point; throws outside the grid's extent. */
  public function bilinearSample(x:Float, y:Float):Float {
    if (!Math.isFinite(x) || !Math.isFinite(y)) throw "Height map sample point must be finite";
    var fCol = (x - originX) / cellSize;
    var fRow = (y - originY) / cellSize;
    if (fCol < 0.0 || fRow < 0.0 || fCol > columns - 1 || fRow > rows - 1)
      throw 'Height map sample ($x, $y) is outside the grid extent';
    var col0 = Math.floor(fCol);
    var row0 = Math.floor(fRow);
    if (col0 >= columns - 1) col0 = columns - 2;
    if (row0 >= rows - 1) row0 = rows - 2;
    var u = fCol - col0;
    var v = fRow - row0;
    var z00 = elevationAt(col0, row0);
    var z10 = elevationAt(col0 + 1, row0);
    var z01 = elevationAt(col0, row0 + 1);
    var z11 = elevationAt(col0 + 1, row0 + 1);
    return z00 * (1.0 - u) * (1.0 - v) + z10 * u * (1.0 - v) + z01 * (1.0 - u) * v + z11 * u * v;
  }

  /** Whether `(x, y)` falls within this grid's extent. */
  public function contains(x:Float, y:Float):Bool {
    var fCol = (x - originX) / cellSize;
    var fRow = (y - originY) / cellSize;
    return fCol >= 0.0 && fRow >= 0.0 && fCol <= columns - 1 && fRow <= rows - 1;
  }

  /**
   * Cut (existing above design)/fill (existing below design) volume between
   * two height maps sharing the same frame and grid geometry. Each of the
   * `(columns - 1) x (rows - 1)` grid cells contributes the average of its
   * four corners' signed `existing - design` difference times the cell's
   * plan area; `skipCell`, if given, omits a cell entirely (its lower-left
   * vertex indices), e.g. for `EarthworkRegion` exclusions.
   */
  public static function volumeBetween(existing:HeightMap, design:HeightMap,
      ?skipCell:(col:Int, row:Int) -> Bool):VolumeResult {
    ensureSameGrid(existing, design);
    var area = existing.cellSize * existing.cellSize;
    var cut = 0.0, fill = 0.0;
    for (row in 0...(existing.rows - 1)) for (col in 0...(existing.columns - 1)) {
      if (skipCell != null && skipCell(col, row)) continue;
      var d00 = existing.elevationAt(col, row) - design.elevationAt(col, row);
      var d10 = existing.elevationAt(col + 1, row) - design.elevationAt(col + 1, row);
      var d01 = existing.elevationAt(col, row + 1) - design.elevationAt(col, row + 1);
      var d11 = existing.elevationAt(col + 1, row + 1) - design.elevationAt(col + 1, row + 1);
      var avg = (d00 + d10 + d01 + d11) * 0.25;
      if (avg > 0.0) cut += avg * area; else fill += -avg * area;
    }
    return new VolumeResult(cut, fill);
  }

  public static function ensureSameGrid(a:HeightMap, b:HeightMap):Void {
    if (a == null || b == null) throw "Height map comparison requires two height maps";
    if (a.frameId != b.frameId || a.columns != b.columns || a.rows != b.rows ||
        Math.abs(a.cellSize - b.cellSize) > 1e-9 ||
        Math.abs(a.originX - b.originX) > 1e-9 || Math.abs(a.originY - b.originY) > 1e-9)
      throw "Height maps must share the same frame and grid geometry to be compared";
  }
}

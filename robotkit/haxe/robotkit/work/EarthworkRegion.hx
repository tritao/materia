package robotkit.work;

/**
 * A grading task: an `existing` and `design` `HeightMap` sharing one grid
 * (see `HeightMap.ensureSameGrid`), exclusion polygons (in the height maps'
 * own XY plane, e.g. an existing utility or structure to leave undisturbed),
 * and the grade tolerance a vertex must be within to count as "at grade".
 */
class EarthworkRegion {
  public final id:String;
  public final existing:HeightMap;
  public final design:HeightMap;
  public final exclusions:Array<Polygon2>;
  public final gradeTolerance:Float;

  public function new(id:String, existing:HeightMap, design:HeightMap,
      ?exclusions:Array<Polygon2>, ?gradeTolerance:Float = 0.02) {
    if (id == null || id.length == 0) throw "Earthwork region requires a non-empty id";
    if (existing == null || design == null) throw "Earthwork region requires existing and design height maps";
    HeightMap.ensureSameGrid(existing, design);
    if (!Math.isFinite(gradeTolerance) || gradeTolerance < 0.0)
      throw "Earthwork region grade tolerance must be finite and non-negative";
    this.id = id;
    this.existing = existing;
    this.design = design;
    this.exclusions = exclusions == null ? [] : exclusions.copy();
    this.gradeTolerance = gradeTolerance;
  }

  /** Signed cut (positive)/fill (negative) still required at a grid vertex to reach design grade. */
  public function deltaAt(col:Int, row:Int):Float return existing.elevationAt(col, row) - design.elevationAt(col, row);

  public function isExcludedAt(col:Int, row:Int):Bool {
    if (exclusions.length == 0) return false;
    var point = new Point2(existing.worldX(col), existing.worldY(row));
    for (exclusion in exclusions) if (exclusion.contains(point)) return true;
    return false;
  }

  /** A vertex is "at grade" once its |existing - design| delta is within tolerance, or it is excluded. */
  public function isAtGrade(col:Int, row:Int):Bool {
    if (isExcludedAt(col, row)) return true;
    return Math.abs(deltaAt(col, row)) <= gradeTolerance;
  }

  /** Fraction of grid vertices (excluded ones count as at grade) currently at grade. */
  public function gradeFraction():Float {
    var total = existing.columns * existing.rows;
    var atGrade = 0;
    for (row in 0...existing.rows) for (col in 0...existing.columns) if (isAtGrade(col, row)) atGrade++;
    return total == 0 ? 1.0 : atGrade / total;
  }

  /** The worst-graded (largest |delta|), non-excluded vertex still needing work; null if all are at grade. */
  public function worstVertex():Null<{col:Int, row:Int, delta:Float}> {
    var best:Null<{col:Int, row:Int, delta:Float}> = null;
    var bestMagnitude = 0.0;
    for (row in 0...existing.rows) for (col in 0...existing.columns) {
      if (isExcludedAt(col, row)) continue;
      var delta = deltaAt(col, row);
      var magnitude = Math.abs(delta);
      if (magnitude > gradeTolerance && magnitude > bestMagnitude) {
        bestMagnitude = magnitude;
        best = { col: col, row: row, delta: delta };
      }
    }
    return best;
  }

  /** Remaining cut/fill volume, ignoring any grid cell touching an exclusion. */
  public function remainingVolume():VolumeResult {
    return HeightMap.volumeBetween(existing, design, function(col:Int, row:Int) {
      return isExcludedAt(col, row) || isExcludedAt(col + 1, row) ||
        isExcludedAt(col, row + 1) || isExcludedAt(col + 1, row + 1);
    });
  }
}

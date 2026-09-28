package stockkit;

/**
  A lattice of +Z rays: ray (i, j) passes through
  (originX + i * spacing, originY + j * spacing). Units are metres.
**/
class StockGrid {
  public final originX:Float;
  public final originY:Float;
  public final spacing:Float;
  public final countX:Int;
  public final countY:Int;

  public function new(originX:Float, originY:Float, spacing:Float, countX:Int,
      countY:Int) {
    if (!(spacing > 0.0) || !Math.isFinite(spacing))
      throw "stock grid spacing must be positive and finite";
    if (countX < 1 || countY < 1) throw "stock grid needs at least one ray each way";
    if (!Math.isFinite(originX) || !Math.isFinite(originY))
      throw "stock grid origin must be finite";
    this.originX = originX;
    this.originY = originY;
    this.spacing = spacing;
    this.countX = countX;
    this.countY = countY;
  }

  /** Rays from (minX, minY) at `spacing`, as many as fit up to (maxX, maxY). */
  public static function covering(minX:Float, minY:Float, maxX:Float, maxY:Float,
      spacing:Float):StockGrid {
    if (!(maxX >= minX) || !(maxY >= minY)) throw "stock grid bounds are inverted";
    return new StockGrid(minX, minY, spacing,
      Math.floor((maxX - minX) / spacing + 1e-9) + 1,
      Math.floor((maxY - minY) / spacing + 1e-9) + 1);
  }

  public function x(i:Int):Float return originX + spacing * i;

  public function y(j:Int):Float return originY + spacing * j;
}

package stockkit;

/**
  Stock compared with a target part ray by ray along one grid's rays.
  Leftover is stock outside the target and gouge is target missing from the
  stock, as lengths along each ray; volumes multiply by the ray spacing
  squared. Z rays see floors and ceilings; X and Y rays see walls, so a wall
  gouged sideways shows on them.
**/
class StockComparison {
  public final grid:StockGrid;
  /** Per ray, i fastest. */
  public final leftover:Array<Float>;
  public final gouge:Array<Float>;
  public final largestLeftover:Array<Float>;
  public final largestGouge:Array<Float>;
  /** Source (history index) of the move that cut the largest gouge on each ray, or -1. */
  public final gougeSource:Array<Int>;

  public function new(grid:StockGrid, leftover:Array<Float>, gouge:Array<Float>,
      largestLeftover:Array<Float>, largestGouge:Array<Float>, gougeSource:Array<Int>) {
    this.grid = grid;
    this.leftover = leftover;
    this.gouge = gouge;
    this.largestLeftover = largestLeftover;
    this.largestGouge = largestGouge;
    this.gougeSource = gougeSource;
  }

  public function leftoverVolume():Float
    return sum(leftover) * grid.spacing * grid.spacing;

  public function gougeVolume():Float
    return sum(gouge) * grid.spacing * grid.spacing;

  /** Deepest single gouge along any ray. */
  public function deepestGouge():Float {
    var deepest = 0.0;
    for (value in largestGouge) deepest = Math.max(deepest, value);
    return deepest;
  }

  /** Thickest single stretch of leftover along any ray. */
  public function thickestLeftover():Float {
    var thickest = 0.0;
    for (value in largestLeftover) thickest = Math.max(thickest, value);
    return thickest;
  }

  /** Sources of moves whose gouges exceed `tolerance` somewhere, each once, in increasing order. */
  public function gougingMoves(tolerance:Float):Array<Int> {
    var sources:Array<Int> = [];
    for (index in 0...gougeSource.length)
      if (largestGouge[index] > tolerance && gougeSource[index] >= 0
          && sources.indexOf(gougeSource[index]) < 0)
        sources.push(gougeSource[index]);
    sources.sort((a, b) -> a - b);
    return sources;
  }

  static function sum(values:Array<Float>):Float {
    var total = 0.0;
    for (value in values) total += value;
    return total;
  }
}

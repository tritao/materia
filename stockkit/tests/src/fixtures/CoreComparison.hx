package fixtures;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import oracle.ExactOracle;
import oracle.SampledReference;
import stockkit.CutMove;
import stockkit.Stock;
import stockkit.StockGrid;
import stockkit.StockInterval;

/**
  Compares StockKit core stock with the references on every ray of the core's
  grid: exactly with the OCCT oracle, and by set containment with the sampled
  reference for moves the oracle refuses (ramps, helices).
**/
class CoreComparison {
  /** Agreement with the oracle's ray depths, in metres. */
  public static inline final DEPTH_TOLERANCE = 1e-9;
  /** Rays across the box's x extent; offsets keep them off the box's round coordinates. */
  static inline final RAYS_ACROSS = 9;

  public static var raysCompared = 0;

  /** Cuts `moves` from a core box stock and compares every ray with the oracle's stock. */
  public static function againstOracle(oracle:Part, moves:Array<CutMove>, minX:Float,
      minY:Float, minZ:Float, maxX:Float, maxY:Float, maxZ:Float, label:String):Void {
    var stock = cutBox(moves, minX, minY, minZ, maxX, maxY, maxZ);
    try {
      var below = minZ - 0.01, length = maxZ - minZ + 0.02;
      var rays = stock.rays(0, 0, stock.grid.countX, stock.grid.countY);
      for (j in 0...stock.grid.countY)
        for (i in 0...stock.grid.countX) {
          var x = stock.grid.x(i), y = stock.grid.y(j);
          var expected = [for (v in ExactOracle.rayIntervals(oracle, new Vector(x, y, below),
            new Vector(0, 0, 1), length)) {lo: below + v.enter, hi: below + v.exit}];
          var actual = spans(rays[j * stock.grid.countX + i]);
          var where = '$label: core ray at ($x, $y)';
          Assert.check(actual.length == expected.length,
            '$where is ${describe(actual)}, oracle ${describe(expected)}');
          for (k in 0...expected.length) {
            Assert.near(actual[k].lo, expected[k].lo, '$where bottom $k', DEPTH_TOLERANCE);
            Assert.near(actual[k].hi, expected[k].hi, '$where top $k', DEPTH_TOLERANCE);
          }
          raysCompared++;
        }
      stock.dispose();
    } catch (error:Dynamic) {
      stock.dispose();
      throw error;
    }
  }

  /**
    Cuts `moves` from a core box stock and checks every ray against the
    sampled reference's bounds on the remaining material.
  **/
  public static function againstSampled(moves:Array<CutMove>, minX:Float, minY:Float,
      minZ:Float, maxX:Float, maxY:Float, maxZ:Float, samples:Int, label:String):Void {
    var stock = cutBox(moves, minX, minY, minZ, maxX, maxY, maxZ);
    try {
      var reference = SampledReference.fromCutMoves(moves);
      var rays = stock.rays(0, 0, stock.grid.countX, stock.grid.countY);
      for (j in 0...stock.grid.countY)
        for (i in 0...stock.grid.countX) {
          var x = stock.grid.x(i), y = stock.grid.y(j);
          var bounds = SampledReference.remaining([{lo: minZ, hi: maxZ}], reference, x, y, samples);
          var actual = spans(rays[j * stock.grid.countX + i]);
          Assert.check(SampledReference.contains(bounds.inner, actual, bounds.outer, 1e-12),
            '$label: core ray at ($x, $y) is ${describe(actual)}, outside the sampled bounds '
            + '${describe(bounds.inner)} .. ${describe(bounds.outer)}');
          raysCompared++;
        }
      stock.dispose();
    } catch (error:Dynamic) {
      stock.dispose();
      throw error;
    }
  }

  /** A core box stock with square ray spacing, cut by `moves`. */
  public static function cutBox(moves:Array<CutMove>, minX:Float, minY:Float, minZ:Float,
      maxX:Float, maxY:Float, maxZ:Float):Stock {
    var spacing = (maxX - minX) / RAYS_ACROSS;
    var originX = minX + 0.37 * spacing, originY = minY + 0.41 * spacing;
    var grid = new StockGrid(originX, originY, spacing, RAYS_ACROSS,
      Math.floor((maxY - originY) / spacing) + 1);
    var stock = Stock.box(grid, minX, minY, minZ, maxX, maxY, maxZ);
    try {
      stock.cut(moves);
    } catch (error:Dynamic) {
      stock.dispose();
      throw error;
    }
    return stock;
  }

  static function spans(intervals:Array<StockInterval>):Array<Span>
    return [for (v in intervals) {lo: v.lo, hi: v.hi}];

  static function describe(spans:Array<Span>):String
    return "[" + [for (s in spans) '${s.lo}..${s.hi}'].join(", ") + "]";
}

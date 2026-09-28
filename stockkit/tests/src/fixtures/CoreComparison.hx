package fixtures;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import oracle.ExactOracle;
import oracle.SampledReference;
import stockkit.CutMove;
import stockkit.Stock;
import stockkit.StockAxis;
import stockkit.StockLattice;
import stockkit.StockInterval;
import toolpathkit.path.PathGeometry;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;

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

  /**
    Cuts `moves` from a tri-dexel core box stock and compares every ray of
    every grid with the oracle's stock.
  **/
  public static function againstOracle(oracle:Part, moves:Array<CutMove>, minX:Float,
      minY:Float, minZ:Float, maxX:Float, maxY:Float, maxZ:Float, label:String):Void {
    var stock = cutBox(moves, minX, minY, minZ, maxX, maxY, maxZ);
    try {
      var min = [minX, minY, minZ], max = [maxX, maxY, maxZ];
      eachRay(stock, (axis, u, v, actual) -> {
        var along = index(axis), before = min[along] - 0.01, length = max[along] - min[along] + 0.02;
        var start = point(axis, before, u, v), direction = point(axis, 1, 0, 0);
        var expected = [for (span in ExactOracle.rayIntervals(oracle, new Vector(start[0], start[1], start[2]),
          new Vector(direction[0], direction[1], direction[2]), length)) {lo: before + span.enter, hi: before + span.exit}];
        var where = '$label: core $axis ray at ($u, $v)';
        Assert.check(actual.length == expected.length, '$where is ${describe(actual)}, oracle ${describe(expected)}');
        for (k in 0...expected.length) {
          Assert.near(actual[k].lo, expected[k].lo, '$where start $k', DEPTH_TOLERANCE);
          Assert.near(actual[k].hi, expected[k].hi, '$where end $k', DEPTH_TOLERANCE);
        }
        raysCompared++;
      });
      stock.dispose();
    } catch (error:Dynamic) {
      stock.dispose();
      throw error;
    }
  }

  /**
    Cuts `moves` from a tri-dexel core box stock and checks every ray of
    every grid against the sampled reference's bounds on the remaining
    material.
  **/
  public static function againstSampled(moves:Array<CutMove>, minX:Float, minY:Float,
      minZ:Float, maxX:Float, maxY:Float, maxZ:Float, samples:Int, label:String):Void {
    var stock = cutBox(moves, minX, minY, minZ, maxX, maxY, maxZ);
    try {
      var reference = SampledReference.fromCutMoves(moves);
      var min = [minX, minY, minZ], max = [maxX, maxY, maxZ];
      eachRay(stock, (axis, u, v, actual) -> {
        var along = index(axis);
        var bounds = SampledReference.remainingAlong(referenceAxis(axis), [{lo: min[along], hi: max[along]}], reference,
          u, v, samples);
        Assert.check(SampledReference.contains(bounds.inner, actual, bounds.outer, 1e-12),
          '$label: core $axis ray at ($u, $v) is ${describe(actual)}, outside the sampled bounds '
          + '${describe(bounds.inner)} .. ${describe(bounds.outer)}');
        raysCompared++;
      });
      stock.dispose();
    } catch (error:Dynamic) {
      stock.dispose();
      throw error;
    }
  }

  /** Calls `visit` with every ray of every grid of `stock`: its axis, (u, v) and intervals. */
  static function eachRay(stock:Stock, visit:(axis:StockAxis, u:Float, v:Float, actual:Array<Span>) -> Void):Void {
    for (axis in [StockAxis.X, StockAxis.Y, StockAxis.Z]) {
      var grid = stock.lattice.grid(axis);
      var rays = stock.rays(0, 0, grid.countU, grid.countV, axis);
      for (j in 0...grid.countV)
        for (i in 0...grid.countU)
          visit(axis, grid.u(i), grid.v(j), spans(rays[j * grid.countU + i]));
    }
  }

  static function index(axis:StockAxis):Int
    return switch axis {
      case X: 0;
      case Y: 1;
      case Z: 2;
    };

  static function referenceAxis(axis:StockAxis):ReferenceAxis
    return switch axis {
      case X: ReferenceAxis.X;
      case Y: ReferenceAxis.Y;
      case Z: ReferenceAxis.Z;
    };

  /** The world point with `along` on `axis` and (u, v) on the other two axes, in order. */
  static function point(axis:StockAxis, along:Float, u:Float, v:Float):Array<Float>
    return switch axis {
      case X: [along, u, v];
      case Y: [u, along, v];
      case Z: [u, v, along];
    };

  /**
    One move's swept material along a lattice of X and Y rays, from the core
    and from the sampled reference, which must bound it.
  **/
  public static function sweepAgainstSampled(profile:CutterProfile, geometry:PathGeometry,
      minX:Float, minY:Float, minZ:Float, maxX:Float, maxY:Float, maxZ:Float, samples:Int,
      label:String):Void {
    var tool = Tool.shaped(1, 0.0, profile);
    var across = 11, up = 9;
    for (axis in [StockAxis.X, StockAxis.Y]) {
      var lo = axis == StockAxis.X ? minY : minX, hi = axis == StockAxis.X ? maxY : maxX;
      for (a in 0...across)
        for (b in 0...up) {
          // Off round coordinates, like a cell-centred lattice.
          var u = lo + (a + 0.37) * (hi - lo) / across;
          var v = minZ + (b + 0.41) * (maxZ - minZ) / up;
          var actual = spans(Stock.sweep(tool, geometry, axis, u, v));
          var reference = axis == StockAxis.X ? ReferenceAxis.X : ReferenceAxis.Y;
          var bounds = SampledReference.sweepAlong(reference, profile, geometry, u, v, samples);
          Assert.check(SampledReference.contains(bounds.inner, actual, bounds.outer, 1e-12),
            '$label: core $axis ray at ($u, $v) sweeps ${describe(actual)}, outside the sampled '
            + 'bounds ${describe(bounds.inner)} .. ${describe(bounds.outer)}');
          raysCompared++;
        }
    }
  }

  /**
    A tri-dexel core box stock cut by `moves`, on a lattice of `RAYS_ACROSS`
    rays across x (at least four across z), offset off the box's round
    coordinates.
  **/
  public static function cutBox(moves:Array<CutMove>, minX:Float, minY:Float, minZ:Float,
      maxX:Float, maxY:Float, maxZ:Float):Stock {
    var spacing = Math.min((maxX - minX) / RAYS_ACROSS, (maxZ - minZ) / 4);
    var originX = minX + 0.37 * spacing, originY = minY + 0.41 * spacing, originZ = minZ + 0.43 * spacing;
    var lattice = new StockLattice(originX, originY, originZ, spacing, Math.floor((maxX - originX) / spacing) + 1,
      Math.floor((maxY - originY) / spacing) + 1, Math.floor((maxZ - originZ) / spacing) + 1);
    var stock = Stock.box(lattice, minX, minY, minZ, maxX, maxY, maxZ);
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

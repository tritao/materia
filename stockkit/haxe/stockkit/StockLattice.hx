package stockkit;

/**
  A lattice of nodes (originX + i * spacing, originY + j * spacing, originZ +
  k * spacing) and a grid of rays through them along each of its axes:
  Z rays through (x_i, y_j), X rays through (y_j, z_k) and Y rays through
  (x_i, z_k). The Z grid is always present; `triDexel` adds X and Y, which
  every cut then updates too. Units are metres.
**/
class StockLattice {
  public final originX:Float;
  public final originY:Float;
  public final originZ:Float;
  public final spacing:Float;
  public final countX:Int;
  public final countY:Int;
  public final countZ:Int;
  public final triDexel:Bool;

  public function new(originX:Float, originY:Float, originZ:Float, spacing:Float, countX:Int,
      countY:Int, countZ:Int, triDexel:Bool = true) {
    if (!(spacing > 0.0) || !Math.isFinite(spacing))
      throw "stock lattice spacing must be positive and finite";
    if (countX < 1 || countY < 1 || countZ < 1) throw "stock lattice needs at least one node each way";
    if (!Math.isFinite(originX) || !Math.isFinite(originY) || !Math.isFinite(originZ))
      throw "stock lattice origin must be finite";
    this.originX = originX;
    this.originY = originY;
    this.originZ = originZ;
    this.spacing = spacing;
    this.countX = countX;
    this.countY = countY;
    this.countZ = countZ;
    this.triDexel = triDexel;
  }

  /**
    Nodes at the centres of `spacing` cells covering the box with one cell
    to spare on every side, so no ray lies in a face of the box and the
    stock's outside is always sampled.
  **/
  public static function covering(minX:Float, minY:Float, minZ:Float, maxX:Float, maxY:Float,
      maxZ:Float, spacing:Float, triDexel:Bool = true):StockLattice {
    if (!(maxX >= minX) || !(maxY >= minY) || !(maxZ >= minZ)) throw "stock lattice bounds are inverted";
    function cells(min:Float, max:Float):Int
      return Std.int(Math.max(1.0, Math.ceil((max - min) / spacing - 1e-9))) + 2;
    return new StockLattice(minX - spacing / 2, minY - spacing / 2, minZ - spacing / 2, spacing,
      cells(minX, maxX), cells(minY, maxY), cells(minZ, maxZ), triDexel);
  }

  public function has(axis:StockAxis):Bool
    return triDexel || axis == Z;

  /** The rays along `axis`. */
  public function grid(axis:StockAxis):StockGrid {
    if (!has(axis)) throw 'stock lattice has no $axis grid';
    return switch axis {
      case X: new StockGrid(X, originY, originZ, spacing, countY, countZ);
      case Y: new StockGrid(Y, originX, originZ, spacing, countX, countZ);
      case Z: new StockGrid(Z, originX, originY, spacing, countX, countY);
    };
  }

  public function x(i:Int):Float return originX + spacing * i;

  public function y(j:Int):Float return originY + spacing * j;

  public function z(k:Int):Float return originZ + spacing * k;
}

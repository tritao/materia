package stockkit;

/**
  The rays of a `StockLattice` along one axis: ray (i, j) passes through
  (u, v) = (originU + i * spacing, originV + j * spacing), where u and v are
  the other two axes in order: (x, y) for Z rays, (y, z) for X rays and
  (x, z) for Y rays. Units are metres.
**/
class StockGrid {
  public final axis:StockAxis;
  public final originU:Float;
  public final originV:Float;
  public final spacing:Float;
  public final countU:Int;
  public final countV:Int;

  public function new(axis:StockAxis, originU:Float, originV:Float, spacing:Float, countU:Int,
      countV:Int) {
    this.axis = axis;
    this.originU = originU;
    this.originV = originV;
    this.spacing = spacing;
    this.countU = countU;
    this.countV = countV;
  }

  public function u(i:Int):Float return originU + spacing * i;

  public function v(j:Int):Float return originV + spacing * j;

  public function rayCount():Int return countU * countV;
}

package stockkit;

/**
  Material along a ray between heights `lo` and `hi`. Each end keeps the
  outward normal of the material there and the source of the move that made
  it: an index into the stock's move history, or `Stock.ORIGINAL` for
  surfaces no move has touched.
**/
class StockInterval {
  public final lo:Float;
  public final hi:Float;
  public final loSource:Int;
  public final hiSource:Int;
  public final loNormal:Array<Float>;
  public final hiNormal:Array<Float>;

  public function new(lo:Float, hi:Float, loSource:Int, hiSource:Int,
      loNormal:Array<Float>, hiNormal:Array<Float>) {
    this.lo = lo;
    this.hi = hi;
    this.loSource = loSource;
    this.hiSource = hiSource;
    this.loNormal = loNormal;
    this.hiNormal = hiNormal;
  }
}

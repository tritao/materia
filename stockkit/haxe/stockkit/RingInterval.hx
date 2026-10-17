package stockkit;

/** Sorted material arc [lo,hi] in [0,2*pi]. Artificial seam boundaries are not surfaces.
    Each real exposed endpoint retains its gash and pass; -1 means original stock. */
class RingInterval {
  public final lo:Float;
  public final hi:Float;
  public final loGash:Int;
  public final loPass:Int;
  public final hiGash:Int;
  public final hiPass:Int;
  public function new(lo:Float, hi:Float, loGash:Int, loPass:Int, hiGash:Int, hiPass:Int) {
    this.lo = lo; this.hi = hi;
    this.loGash = loGash; this.loPass = loPass;
    this.hiGash = hiGash; this.hiPass = hiPass;
  }
}

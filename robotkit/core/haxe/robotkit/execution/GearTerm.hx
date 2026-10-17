package robotkit.execution;

/** SI encoder timing and count size, explicitly supplied by the installed controller/driver. */
class GearTerm {
  public final master:Int;
  public final ratio:Float;
  public final latencyNs:Int;
  public final quantum:Float;
  public final extrapolate:Bool;
  public function new(master:Int, ratio:Float, latencyNs:Int, quantum:Float, extrapolate:Bool = true) {
    if (master < 0 || !Math.isFinite(ratio) || ratio == 0 || latencyNs < 0 || latencyNs > 1000000000 ||
        !Math.isFinite(quantum) || quantum < 0) throw "Invalid electronic gear encoder term";
    this.master = master; this.ratio = ratio; this.latencyNs = latencyNs;
    this.quantum = quantum; this.extrapolate = extrapolate;
  }
}

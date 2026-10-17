package toolpathkit.path;

/** A master joint by stable machine name, rather than a mechanical transmission. */
class GearTerm {
  public final master:String;
  public final ratio:Float;
  public function new(master:String, ratio:Float) {
    if (master == null || master.length == 0 || !Math.isFinite(ratio) || ratio == 0)
      throw "Electronic gear needs a named master and finite nonzero ratio";
    this.master = master; this.ratio = ratio;
  }
}

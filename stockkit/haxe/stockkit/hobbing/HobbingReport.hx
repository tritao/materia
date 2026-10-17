package stockkit.hobbing;
/** Numerical convergence is observed on the sampled circles, not a machine accuracy grade. */
class HobbingReport {
  public final passes:Int;public final poses:Int;public final refinement:Int;
  public final maxArcChange:Float;
  public function new(passes:Int,poses:Int,refinement:Int,maxArcChange:Float) {
    this.passes=passes;this.poses=poses;this.refinement=refinement;this.maxArcChange=maxArcChange;
  }
}

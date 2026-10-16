package motionkit.robot;

/** Joint-position allowances; the shared native merger derives derivative bounds for each actual candidate duration.
 * With e(0)=e(T)=0, |e'|<=2*epsilon/T or |e''|<=8*epsilon/T^2 each implies |e|<=epsilon
 * for a continuously differentiable error (piecewise bounded second derivative suffices).
 * The native merger also checks position error directly, including numerical roundoff.
 */
class MergeErrorBudget {
  public final position:Array<Float>;
  public final seconds:Float;
  public function new(position:Array<Float>,seconds:Float) {
    if(position==null || position.length==0 || !Math.isFinite(seconds) || seconds<=0 || seconds>=Math.pow(2,63)*1e-9)
      throw "Merge budget needs joint allowances and a positive window";
    this.position=position.copy();this.seconds=seconds;
    for(value in position)if(!Math.isFinite(value) || value<0)throw "Merge allowances must be finite and nonnegative";

  }
}

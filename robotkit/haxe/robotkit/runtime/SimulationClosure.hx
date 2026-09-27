package robotkit.runtime;

/** Closed assembly edge between two links in an existing tree. */
class SimulationClosure {
  public final parentLink:Int;
  public final childLink:Int;
  public final type:Int;
  public final anchorParent:Array<Float>;
  public final axisParent:Array<Float>;

  public function new(parentLink:Int, childLink:Int, type:Int,
      anchorParent:Array<Float>, axisParent:Array<Float>) {
    this.parentLink = parentLink;
    this.childLink = childLink;
    this.type = type;
    this.anchorParent = anchorParent.copy();
    this.axisParent = axisParent.copy();
  }
}

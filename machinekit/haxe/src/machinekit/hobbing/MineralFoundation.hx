package machinekit.hobbing;

/** Compatibility adapter retaining the hobber layout identity on the shared foundation. */
class MineralFoundation extends machinekit.structural.MineralFoundation {
  public final kind:String;
  public function new(kind:String){
    super(FoundationLayout.bodyValues(kind),FoundationLayout.placements(kind),kind=="column"?"basePlate":null);
    this.kind=kind;
  }
}

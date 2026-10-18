package machinekit.hobbing;

/** Hobber reference layout; shared casting and insert geometry lives in structural. */
class MineralCastColumn extends machinekit.structural.MineralFoundation {
  public function new() super(FoundationLayout.bodyValues("column"), FoundationLayout.placements("column"), "basePlate");
}

package machinekit.hobbing;

/** Hobber reference layout; shared casting and insert geometry lives in structural. */
class MineralCastBed extends machinekit.structural.MineralFoundation {
  public function new() super(FoundationLayout.bodyValues("bed"), FoundationLayout.placements("bed"));
}

package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastBed extends machinekit.structural.MineralFoundation {
  public function new() {
    var source = HobberFoundationSizing.resolve("bed");
    super(source.body, source.placements, null, null, source.liftingPoints);
  }
}

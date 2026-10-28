package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastColumn extends machinekit.structural.MineralFoundation {
  public function new() {
    var source = HobberFoundationSizing.resolve("column");
    super(source.body, source.placements, "basePlate", null, source.liftingPoints);
  }
}

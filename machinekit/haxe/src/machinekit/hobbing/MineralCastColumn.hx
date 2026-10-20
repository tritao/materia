package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastColumn extends machinekit.structural.MineralFoundation {
	public function new()
		super(FoundationLayout.bodyValues("column"), FoundationLayout.placements("column"), "basePlate");
}

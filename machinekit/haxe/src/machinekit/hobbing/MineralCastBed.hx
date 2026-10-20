package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastBed extends machinekit.structural.MineralFoundation {
	public function new()
		super(FoundationLayout.bodyValues("bed"), FoundationLayout.placements("bed"), null);
}

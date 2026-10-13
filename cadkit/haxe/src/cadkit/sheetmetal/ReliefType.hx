package cadkit.sheetmetal;

/** Bend-end relief outline; dimensions are owned separately by the flange feature. */
@:wire enum ReliefType {
	@:id(1) None;
	@:id(2) Rectangular;
	@:id(3) Obround;
}

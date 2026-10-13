package cadkit.sheetmetal;

/** Relief dimensions are either millimetres or explicit multiples of the sheet thickness. */
@:wire enum ReliefSizing {
	@:id(1) Absolute;
	@:id(2) ThicknessRatio;
}

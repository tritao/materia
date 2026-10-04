package cadkit.sheetmetal;

/** Manufacturability findings are independent of fatal graph validation. */
enum SheetCheckKind {
	FlatOverlap;
	ShortFlange;
	CutNearBend;
	MissingRelief;
	SmallRadius;
	AlongGrain;
}

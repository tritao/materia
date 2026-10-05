package cadkit.sheetmetal;

/** Exact corner relief shape; size is the circle diameter or square side in the parent blank. */
@:wire enum CornerRelief {
	@:id(1) Round;
	@:id(2) Square;
}

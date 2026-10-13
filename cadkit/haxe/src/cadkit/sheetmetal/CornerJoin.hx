package cadkit.sheetmetal;

/** Corner seam arrangement; overlap names the flange that extends beyond its mate's thickness. */
@:wire enum CornerJoin {
	@:id(1) Gap;
	@:id(2) OverlapFirst;
	@:id(3) OverlapSecond;
}

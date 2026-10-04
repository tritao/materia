package cadkit.sheetmetal;

/** Alignment of the bend's parent tangent to the original picked boundary. */
@:wire enum FlangePosition {
	/** The virtual outside sharp corner stays at the picked edge. */
	@:id(1) MaterialInside;
	/** The virtual inside sharp corner stays at the picked edge. */
	@:id(2) MaterialOutside;
	/** The bend begins at the picked edge; the original straight wall is unchanged. */
	@:id(3) BendOutside;
	/** Move the parent tangent by the authored offset, positive outward. */
	@:id(4) Offset;
}

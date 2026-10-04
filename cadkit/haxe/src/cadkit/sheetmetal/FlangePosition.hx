package cadkit.sheetmetal;

/** Alignment of the bend's parent tangent to the original picked boundary. */
enum FlangePosition {
	/** The virtual outside sharp corner stays at the picked edge. */
	MaterialInside;
	/** The virtual inside sharp corner stays at the picked edge. */
	MaterialOutside;
	/** The bend begins at the picked edge; the original straight wall is unchanged. */
	BendOutside;
	/** Move the parent tangent by the authored offset, positive outward. */
	Offset;
}

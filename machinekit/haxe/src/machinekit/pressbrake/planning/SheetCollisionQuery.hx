package machinekit.pressbrake.planning;

/** Narrow convex material boundary; curved-contact refinement remains in the press queries. */
interface SheetCollisionQuery {
	function overlap(a:ConvexPrism, b:ConvexPrism):Bool;
	/** Signed separation in mm; a positive support gap is a lower bound on material distance. */
	function separation(a:ConvexPrism, b:ConvexPrism):Float;
}

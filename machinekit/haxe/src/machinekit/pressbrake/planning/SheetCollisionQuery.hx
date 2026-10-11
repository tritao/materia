package machinekit.pressbrake.planning;

/** Narrow convex material boundary; curved-contact refinement remains in the press queries. */
interface SheetCollisionQuery {
	function overlap(a:ConvexPrism, b:ConvexPrism):Bool;
	/** Signed convex distance in mm; positive separation certifies material clearance. */
	function separation(a:ConvexPrism, b:ConvexPrism):Float;
}

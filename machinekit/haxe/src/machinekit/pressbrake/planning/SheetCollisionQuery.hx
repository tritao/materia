package machinekit.pressbrake.planning;

/** Narrow collision boundary. The interim prism implementation is replaced when CollisionKit lands on main. */
interface SheetCollisionQuery {
	function overlap(a:ConvexPrism, b:ConvexPrism):Bool;
	/** Largest signed support gap in mm; a positive value is a lower bound on material distance. */
	function separation(a:ConvexPrism, b:ConvexPrism):Float;
}

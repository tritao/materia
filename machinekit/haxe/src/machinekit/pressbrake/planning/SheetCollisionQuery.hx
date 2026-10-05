package machinekit.pressbrake.planning;

/** Narrow collision boundary. The interim prism implementation is replaced when CollisionKit lands on main. */
interface SheetCollisionQuery {
	function overlap(a:ConvexPrism, b:ConvexPrism):Bool;
}
